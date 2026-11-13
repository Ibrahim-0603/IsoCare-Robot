"""Run the odometry core against the fake ESP32 in simulated time.

The fake's physics are made up (2.0 ticks/mm, 150 mm wheelbase), so these tests check
that the odometry agrees with the fake's own ticks and gyro, not that it matches a real robot.
"""

import math
from pathlib import Path
import sys

import fake_esp32
from isocare_serial import codec
from test_bridge_with_fake import Loop

try:
    from isocare_navigation import odometry_core
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'isocare_navigation'))
    from isocare_navigation import odometry_core


def feed(core, loop, start=0):
    """Feed the loop's SensorData messages from index start into the core. Return the new end."""
    messages = loop.msgs(codec.SensorData)
    for m in messages[start:]:
        core.update(m.t_ms, m.left_ticks, m.right_ticks, m.gyro_z)
    return len(messages)


def test_straight_drive_matches_the_fake_ticks():
    """Odometry x equals the distance the fake's own ticks say it travelled."""
    loop = Loop()
    loop.run(5.0, cmd_vel=(0.2, 0.0))
    core = odometry_core.OdometryCore()
    feed(core, loop)
    last = loop.msgs(codec.SensorData)[-1]
    expected_m = last.left_ticks / fake_esp32.TICKS_PER_MM / 1000.0
    assert 0.95 < core.pose.x_m < 1.0            # 5 s at 0.2 m/s minus the wheel lag
    assert abs(core.pose.x_m - expected_m) < 0.01
    assert abs(core.pose.y_m) < 1e-6 and abs(core.pose.heading_rad) < 1e-6


def test_turn_heading_agrees_between_gyro_and_wheels():
    """Gyro heading and wheel-difference heading agree: units and signs are consistent."""
    loop = Loop()
    loop.run(3.2, cmd_vel=(0.0, 0.5))
    loop.run(0.5)                                  # let the wheels stop
    core = odometry_core.OdometryCore()
    feed(core, loop)
    last = loop.msgs(codec.SensorData)[-1]
    wheel_heading = ((last.right_ticks - last.left_ticks) / fake_esp32.TICKS_PER_MM
                     / fake_esp32.WHEELBASE_MM)
    # 3.2 s at 0.5 rad/s is 1.6 rad; the bridge keeps sending the last command for 0.3 s more.
    assert 1.5 < core.pose.heading_rad < 1.8
    assert abs(core.pose.heading_rad - wheel_heading) < 0.03


def test_driving_a_circle_ends_near_the_start():
    """0.2 m/s with 0.4 rad/s is a circle of 0.5 m radius; one lap returns to the start."""
    loop = Loop()
    loop.run(2 * math.pi / 0.4, cmd_vel=(0.2, 0.4))
    loop.run(0.5)
    core = odometry_core.OdometryCore()
    feed(core, loop)
    assert math.hypot(core.pose.x_m, core.pose.y_m) < 0.08


def test_esp32_soft_reset_does_not_move_the_pose():
    """A real reboot (SOFT_RESET) restarts clock and ticks; the pose carries on from before."""
    loop = Loop()
    core = odometry_core.OdometryCore()
    loop.run(2.0, cmd_vel=(0.2, 0.0))
    loop.run(0.5)
    seen = feed(core, loop)
    before = core.pose
    first_leg = loop.msgs(codec.SensorData)[-1].left_ticks
    loop.core.submit(codec.SystemCmd(codec.SystemCommand.SOFT_RESET))
    loop.run(0.5)
    seen = feed(core, loop, seen)
    assert core.reboots == 1
    assert abs(core.pose.x_m - before.x_m) < 0.01     # at most the lost 20 ms worth
    loop.run(2.0, cmd_vel=(0.2, 0.0))
    loop.run(0.5)
    feed(core, loop, seen)
    second_leg = loop.msgs(codec.SensorData)[-1].left_ticks
    expected_m = (first_leg + second_leg) / fake_esp32.TICKS_PER_MM / 1000.0
    assert abs(core.pose.x_m - expected_m) < 0.02
