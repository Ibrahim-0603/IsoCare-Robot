"""Tests for isocare_navigation.odometry_core. Plain pytest, no ROS needed."""

import math
from pathlib import Path
import sys

try:
    from isocare_navigation import odometry_core as oc
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_navigation import odometry_core as oc
import pytest

TPM = 2.0   # ticks per mm used in these tests (the default)


def make_core(**kwargs):
    """Return a core with a baseline sample at t=0 already taken."""
    core = oc.OdometryCore(oc.OdometryConfig(**kwargs))
    assert core.update(0, 0, 0, 0) == oc.FIRST
    return core


def drive(core, seconds, speed_mm_s=0.0, turn_deg_s=0.0, t0_ms=0, ticks0=(0, 0), period_ms=20):
    """Feed ideal samples: both wheels at speed_mm_s, gyro at turn_deg_s. Return end state."""
    t_ms, left, right = t0_ms, float(ticks0[0]), float(ticks0[1])
    steps = int(round(seconds * 1000 / period_ms))
    for _ in range(steps):
        t_ms += period_ms
        left += speed_mm_s * TPM * period_ms / 1000.0
        right += speed_mm_s * TPM * period_ms / 1000.0
        core.update(t_ms, round(left), round(right), round(turn_deg_s * 100))
    return t_ms, (round(left), round(right))


def test_first_sample_is_only_a_baseline():
    """The first sample sets the baseline; the pose stays at the origin even with big ticks."""
    core = oc.OdometryCore()
    assert core.update(123456, 90000, 90000, 500) == oc.FIRST
    assert core.pose == oc.Pose()


def test_straight_line_distance_from_ticks():
    """2 m at 200 mm/s: 10 s, x ends near 2.0 m, y and heading stay 0."""
    core = make_core()
    drive(core, 10.0, speed_mm_s=200.0)
    assert core.pose.x_m == pytest.approx(2.0, abs=0.002)
    assert core.pose.y_m == pytest.approx(0.0, abs=1e-9)
    assert core.pose.heading_rad == pytest.approx(0.0, abs=1e-9)
    assert core.pose.v_m_s == pytest.approx(0.2, abs=0.005)


def test_ticks_per_mm_scales_the_distance():
    """The same ticks mean half the distance when ticks_per_mm is doubled."""
    core = make_core(ticks_per_mm=4.0)
    drive(core, 10.0, speed_mm_s=200.0)     # ticks generated at 2 per mm
    assert core.pose.x_m == pytest.approx(1.0, abs=0.002)


def test_spin_on_the_spot_uses_the_gyro():
    """90 deg/s for 1 s turns 90 degrees without moving."""
    core = make_core()
    drive(core, 1.0, turn_deg_s=90.0)
    assert core.pose.heading_rad == pytest.approx(math.pi / 2, abs=1e-6)
    assert core.pose.x_m == pytest.approx(0.0, abs=1e-9)
    assert core.pose.omega_rad_s == pytest.approx(math.pi / 2, abs=1e-3)


def test_clockwise_spin_is_negative():
    """Negative gyro values turn clockwise (negative heading)."""
    core = make_core()
    drive(core, 1.0, turn_deg_s=-45.0)
    assert core.pose.heading_rad == pytest.approx(-math.pi / 4, abs=1e-6)


def test_heading_wraps_after_a_full_turn_and_a_bit():
    """450 degrees of turning reads as 90 degrees, never above pi."""
    core = make_core()
    drive(core, 5.0, turn_deg_s=90.0)
    assert core.pose.heading_rad == pytest.approx(math.pi / 2, abs=1e-6)
    assert -math.pi < core.pose.heading_rad <= math.pi


def test_circle_returns_to_the_start():
    """Driving a full circle (v=200 mm/s, 36 deg/s for 10 s) ends where it began."""
    core = make_core()
    drive(core, 10.0, speed_mm_s=200.0, turn_deg_s=36.0)
    assert core.pose.x_m == pytest.approx(0.0, abs=0.003)
    assert core.pose.y_m == pytest.approx(0.0, abs=0.003)


def test_midpoint_heading_beats_the_old_heading_on_a_coarse_step():
    """One big step while turning lands on the true arc chord, not off it."""
    core = make_core()
    # 100 ms step: 100 mm forward while turning 90 deg/s (9 degrees).
    core.update(100, 200, 200, 9000)
    chord = 2 * (0.1 / math.radians(9)) * math.sin(math.radians(9) / 2)   # exact arc chord
    exact_x = chord * math.cos(math.radians(4.5))
    assert core.pose.x_m == pytest.approx(exact_x, abs=2e-4)
    assert core.pose.y_m > 0.0     # the old heading (0 deg) would give exactly 0


def test_dt_comes_from_the_esp32_clock_only():
    """The result depends on esp_time_ms alone: there is no other clock input."""
    fast = make_core()
    fast.update(10, 0, 0, 9000)
    slow = make_core()
    slow.update(20, 0, 0, 9000)
    assert slow.pose.heading_rad == pytest.approx(2 * fast.pose.heading_rad)


def test_duplicate_timestamp_is_ignored():
    """Two samples with the same ESP32 time are not integrated twice."""
    core = make_core()
    core.update(20, 40, 40, 0)
    before = core.pose
    assert core.update(20, 40, 40, 0) == oc.DUPLICATE
    assert core.pose == before


def test_esp32_reboot_keeps_the_pose_and_takes_a_new_baseline():
    """Clock and ticks restart: no teleport, and driving continues from the old pose."""
    core = make_core()
    t_ms, ticks = drive(core, 5.0, speed_mm_s=200.0)
    before = core.pose
    assert before.x_m == pytest.approx(1.0, abs=0.002)
    assert core.update(40, 4, 4, 0) == oc.REBOOT      # ESP32 restarted, ticks back near 0
    assert core.reboots == 1
    assert (core.pose.x_m, core.pose.y_m, core.pose.heading_rad) == (
        before.x_m, before.y_m, before.heading_rad)
    assert core.pose.v_m_s == 0.0
    drive(core, 5.0, speed_mm_s=200.0, t0_ms=40, ticks0=(4, 4))
    assert core.pose.x_m == pytest.approx(2.0, abs=0.003)


def test_reboot_with_a_garbage_first_sample_does_not_jump():
    """The first sample after a reboot is never used as a step, whatever its ticks."""
    core = make_core()
    drive(core, 1.0, speed_mm_s=200.0)
    before = core.pose.x_m
    core.update(10, 9999, 9999, 0)
    assert core.pose.x_m == before


def test_long_gap_is_not_integrated():
    """A silence longer than max_gap_s gives a new baseline, not a huge step."""
    core = make_core()
    drive(core, 1.0, speed_mm_s=200.0)
    before = core.pose
    assert core.update(1000 + 600, 2000, 2000, 9000) == oc.GAP
    assert core.gaps == 1
    assert core.pose.x_m == before.x_m
    assert core.pose.heading_rad == before.heading_rad


def test_gap_limit_is_a_parameter():
    """The same 0.6 s silence is a normal step when max_gap_s allows it."""
    core = make_core(max_gap_s=1.0)
    assert core.update(600, 240, 240, 0) == oc.STEP
    assert core.pose.x_m == pytest.approx(0.12)


def test_no_gyro_falls_back_to_the_wheel_difference():
    """With imu_ok=False, the wheel difference turns the robot: 90 mm gap over 150 mm base."""
    core = make_core(wheelbase_mm=150.0)
    # right wheel advances 15 mm more than the left: turn = 15 / 150 = 0.1 rad
    core.update(100, 0, 30, 99999, imu_ok=False)   # the gyro value must be ignored
    assert core.pose.heading_rad == pytest.approx(0.1, abs=1e-9)
    assert core.pose.omega_rad_s == pytest.approx(1.0)


def test_gyro_is_used_when_imu_is_ok_even_if_wheels_disagree():
    """With a good gyro the wheel difference is ignored (slip does not turn the robot)."""
    core = make_core()
    core.update(100, 0, 30, 0, imu_ok=True)
    assert core.pose.heading_rad == 0.0


def test_negative_ticks_move_backwards():
    """Driving in reverse gives negative x."""
    core = make_core()
    drive(core, 5.0, speed_mm_s=-100.0)
    assert core.pose.x_m == pytest.approx(-0.5, abs=0.002)


@pytest.mark.parametrize('field', ['ticks_per_mm', 'wheelbase_mm', 'max_gap_s'])
@pytest.mark.parametrize('bad', [0.0, -1.0, math.nan, math.inf])
def test_bad_config_is_rejected(field, bad):
    """Zero, negative or non-finite settings raise instead of producing nonsense."""
    with pytest.raises(ValueError):
        oc.OdometryConfig(**{field: bad})


def test_quaternion_is_a_unit_rotation_about_z():
    """Yaw 90 degrees gives z = w = sin(45) = cos(45); yaw 0 gives the identity."""
    assert oc.yaw_to_quaternion(0.0) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = oc.yaw_to_quaternion(math.pi / 2)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('angle,expected', [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi / 2, -math.pi / 2),
    (3 * math.pi / 2, -math.pi / 2), (-5 * math.pi / 2, -math.pi / 2)])
def test_wrap_angle(angle, expected):
    """Angles are wrapped to (-pi, pi]."""
    assert oc.wrap_angle(angle) == pytest.approx(expected)
