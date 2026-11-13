"""ROS-free brain of the odometry node.

The node in odometry.py only moves data between ROS and this module. Every
decision lives here, so it can be tested with plain pytest and against
tools/fake_esp32.py in simulated time. Nothing in this module reads a clock:
time comes from the ESP32's own millisecond counter inside each sample.

What it keeps: the robot's pose (x, y, heading) measured from where it started.
Each new sample moves the pose by one small step:
    distance  from the average of the two wheels' tick change
    turn      from the gyro rate times dt (heading source), or, if the gyro
              is faulty, from the difference between the wheels
    direction the step is applied along the MIDPOINT heading (old heading
              plus half the turn), which is more accurate while turning
"""

from dataclasses import dataclass
import math

# What update() did with a sample.
FIRST = 'first'          # first sample: only a baseline, the pose stays at the origin
STEP = 'step'            # a normal step was applied
DUPLICATE = 'duplicate'  # same ESP32 timestamp as before: ignored
REBOOT = 'reboot'        # ESP32 clock went backwards: new baseline, pose kept
GAP = 'gap'              # too long since the last sample: new baseline, pose kept


@dataclass
class OdometryConfig:
    """Tunable settings. The node fills these from ROS parameters."""

    ticks_per_mm: float = 2.0      # placeholder (matches the fake ESP32) until calibrated
    wheelbase_mm: float = 150.0    # placeholder; only used for the no-gyro fallback
    max_gap_s: float = 0.5         # a longer silence than this is not integrated

    def __post_init__(self):
        """Reject values that would make the maths meaningless."""
        if not (math.isfinite(self.ticks_per_mm) and self.ticks_per_mm > 0.0):
            raise ValueError('ticks_per_mm must be a positive number')
        if not (math.isfinite(self.wheelbase_mm) and self.wheelbase_mm > 0.0):
            raise ValueError('wheelbase_mm must be a positive number')
        if not (math.isfinite(self.max_gap_s) and self.max_gap_s > 0.0):
            raise ValueError('max_gap_s must be a positive number')


@dataclass(frozen=True)
class Pose:
    """Where the robot is: metres and radians, counter-clockwise positive."""

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0   # wrapped to (-pi, pi]
    v_m_s: float = 0.0         # forward speed over the last step
    omega_rad_s: float = 0.0   # turn rate over the last step


def wrap_angle(angle_rad):
    """Return the angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_to_quaternion(yaw_rad):
    """Return (x, y, z, w) of a rotation by yaw_rad around the vertical axis."""
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class OdometryCore:
    """Turn a stream of ESP32 samples into a pose."""

    def __init__(self, config=None):
        """Start at the origin, facing along +x, with no baseline yet."""
        self.config = config if config is not None else OdometryConfig()
        self.pose = Pose()
        self.reboots = 0     # how many ESP32 reboots were detected
        self.gaps = 0        # how many too-long silences were skipped
        self._last = None    # (esp_time_ms, left_ticks, right_ticks) of the previous sample

    def update(self, esp_time_ms, left_ticks, right_ticks, gyro_z_centideg_s, imu_ok=True):
        """Apply one sample and return what happened (FIRST, STEP, DUPLICATE, REBOOT, GAP).

        esp_time_ms and the ticks are the ESP32's own cumulative values. imu_ok is
        False while the ESP32 reports a gyro fault; the turn then comes from the wheels.
        """
        sample = (esp_time_ms, left_ticks, right_ticks)
        if self._last is None:
            self._last = sample
            return FIRST
        last_time, last_left, last_right = self._last
        dt_ms = esp_time_ms - last_time
        if dt_ms < 0:
            self.reboots += 1
            self._last = sample
            self.pose = Pose(self.pose.x_m, self.pose.y_m, self.pose.heading_rad)
            return REBOOT
        if dt_ms == 0:
            return DUPLICATE
        dt = dt_ms / 1000.0
        if dt > self.config.max_gap_s:
            self.gaps += 1
            self._last = sample
            self.pose = Pose(self.pose.x_m, self.pose.y_m, self.pose.heading_rad)
            return GAP

        d_left_mm = (left_ticks - last_left) / self.config.ticks_per_mm
        d_right_mm = (right_ticks - last_right) / self.config.ticks_per_mm
        distance_m = (d_left_mm + d_right_mm) / 2.0 / 1000.0
        if imu_ok:
            turn = math.radians(gyro_z_centideg_s / 100.0) * dt
        else:
            turn = (d_right_mm - d_left_mm) / self.config.wheelbase_mm
        midpoint = self.pose.heading_rad + turn / 2.0
        self.pose = Pose(
            x_m=self.pose.x_m + distance_m * math.cos(midpoint),
            y_m=self.pose.y_m + distance_m * math.sin(midpoint),
            heading_rad=wrap_angle(self.pose.heading_rad + turn),
            v_m_s=distance_m / dt,
            omega_rad_s=turn / dt,
        )
        self._last = sample
        return STEP
