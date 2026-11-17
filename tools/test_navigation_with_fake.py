"""Run odometry + navigator + route planner against the real bridge core and the fake ESP32.

Simulated time, no ROS. The wheels, lag, ticks and gyro come from the fake (2 ticks/mm, 150 mm
wheelbase), odometry is the real OdometryCore, and the planner is the real route core with the
demo marker table. So this checks that the pieces agree on units, signs and frames, not that
the numbers match a real robot.
"""

import math
from pathlib import Path
import sys

import fake_esp32
from isocare_serial import codec
from test_bridge_with_fake import Loop

ROOT = Path(__file__).resolve().parents[1]
try:
    from isocare_navigation import nav_core, odometry_core, route_core
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(ROOT / 'src' / 'isocare_navigation'))
    from isocare_navigation import nav_core, odometry_core, route_core

DEMO = ROOT / 'src' / 'isocare_bringup' / 'config' / 'markers.yaml'
STEP_S = 0.05


class Rig:
    """The robot side is real (bridge + fake + odometry); the navigator is under test."""

    def __init__(self, **config):
        """Park the robot at the dock pose and give odometry that pose, like startup does."""
        self.loop = Loop()
        self.graph = route_core.MarkerMap.load_file(DEMO)
        self.nav = nav_core.NavCore(nav_core.NavConfig(**config))
        self.odo = odometry_core.OdometryCore()
        dock = self.graph.markers['dock']
        self.odo.set_pose(*nav_core.dock_pose(dock.x_m, dock.y_m, dock.heading_rad,
                                              self.nav.config.dock_standoff_m))
        self.cmd = (0.0, 0.0)
        self.done = None
        self.log = []
        self.plans = []
        self._seen = 0

    def goto(self, target):
        """Give the navigator a goal."""
        self._apply(self.nav.start(target, self.loop.now))

    def step(self):
        """Feed odometry, run one navigator cycle, send its command, advance the fake."""
        sensors = self.loop.msgs(codec.SensorData)
        for m in sensors[self._seen:]:
            self.odo.update(m.t_ms, m.left_ticks, m.right_ticks, m.gyro_z)
        self._seen = len(sensors)
        statuses = self.loop.msgs(codec.Status)
        flags = statuses[-1].flags if statuses else codec.StatusFlag(0)
        front = sensors[-1].range_front_mm if sensors else None
        pose = self.odo.pose
        sample = nav_core.Sample(pose.x_m, pose.y_m, pose.heading_rad, front,
                                 codec.StatusFlag.ESTOP in flags,
                                 codec.StatusFlag.OBST_STOP in flags, True)
        self._apply(self.nav.update(sample, self.loop.now))
        self.loop.core.set_cmd_vel(self.cmd[0], self.cmd[1], self.loop.now)
        self.loop.run(STEP_S)

    def run(self, limit_s=200.0):
        """Run until the goal ends."""
        end = self.loop.now + limit_s
        while self.done is None:
            assert self.loop.now < end, f'no result after {limit_s} s, state {self.nav.state}'
            self.step()
        return self.done

    def _apply(self, commands):
        """Carry out the core's commands like the node does."""
        for command in commands:
            if isinstance(command, nav_core.CmdVel):
                self.cmd = (command.linear, command.angular)
            elif isinstance(command, nav_core.Done):
                self.done = command
            elif isinstance(command, nav_core.Log):
                self.log.append(command.text)
            elif isinstance(command, nav_core.PlanRequest):
                self.plans.append(command)
                self._apply(self.nav.on_plan(self._plan(command), self.loop.now))

    def _plan(self, request):
        """Answer a plan request with the real planner core."""
        try:
            route = self.graph.plan(request.from_name, request.to_name, request.blocked)
        except route_core.RouteError as error:
            return nav_core.PlanResult(False, str(error))
        return nav_core.PlanResult(True, '', route.names, tuple(m.x_m for m in route.markers),
                                   tuple(m.y_m for m in route.markers),
                                   tuple(m.heading_rad for m in route.markers))

    def range_to(self, name):
        """Return the odometry distance from the robot to a marker."""
        marker = self.graph.markers[name]
        return math.hypot(marker.x_m - self.odo.pose.x_m, marker.y_m - self.odo.pose.y_m)


def test_dock_to_bed_a_through_the_real_fake_dynamics():
    """Wheel lag, ticks and gyro from the fake: the robot reaches A at the standoff, facing it."""
    rig = Rig()
    rig.goto('A')
    assert rig.run() == nav_core.Done(True)
    assert abs(rig.range_to('A') - 0.5) < 0.12
    facing = rig.graph.markers['A'].heading_rad + math.pi
    assert abs(nav_core.wrap_angle(rig.odo.pose.heading_rad - facing)) < math.radians(10)
    assert rig.nav.last_marker == 'A'
    # the fake's own wheel ticks agree with what odometry thinks it travelled
    sensors = rig.loop.msgs(codec.SensorData)
    assert sensors[-1].left_ticks > 3 * 1000 * fake_esp32.TICKS_PER_MM    # drove at least 3 m


def test_signs_and_units_agree_the_robot_really_turns_toward_the_target():
    """Starting facing away from the junction: the heading flips by about 180 degrees first."""
    rig = Rig()
    rig.goto('A')
    for _ in range(int(9.0 / STEP_S)):                # 9 s: the turn is done, driving has begun
        rig.step()
    assert abs(rig.odo.pose.heading_rad) < math.radians(15)     # now pointing along +x
    assert rig.odo.pose.x_m > 0.6


def test_obstacle_ahead_stops_it_then_it_resumes_when_the_range_clears():
    """The fake's real 15 cm stop: the navigator waits, then carries on when the range clears."""
    rig = Rig()
    rig.goto('A')
    for _ in range(int(11.0 / STEP_S)):
        rig.step()
    rig.loop.esp.command('range front 100', rig.loop.now)
    while rig.nav.state != nav_core.BLOCKED:
        rig.step()
        assert rig.loop.now < 40.0
    held = rig.odo.pose.x_m
    for _ in range(int(3.0 / STEP_S)):
        rig.step()
    assert abs(rig.odo.pose.x_m - held) < 0.05                  # it stayed put
    rig.loop.esp.command('range front 2000', rig.loop.now)
    assert rig.run() == nav_core.Done(True)
    assert any('resuming' in line for line in rig.log)


def test_blocked_robot_can_turn_away_because_the_stop_is_forward_only():
    """Obstacle at 10 cm, new goal behind the robot: the fake lets it spin, so it turns away."""
    rig = Rig()
    rig.goto('A')
    for _ in range(int(11.0 / STEP_S)):
        rig.step()
    rig.loop.esp.command('range front 100', rig.loop.now)     # the fake's range does not move
    while rig.nav.state != nav_core.BLOCKED:
        rig.step()
        assert rig.loop.now < 40.0
    rig._apply(rig.nav.cancel(rig.loop.now))
    rig.done = None
    rig.goto('dock')                                          # the dock is behind the robot
    start_heading = rig.odo.pose.heading_rad
    for _ in range(int(4.0 / STEP_S)):
        rig.step()
    turned = abs(nav_core.wrap_angle(rig.odo.pose.heading_rad - start_heading))
    assert turned > math.radians(60)                          # it really spun away


def test_estop_during_the_drive_ends_the_goal_and_the_fake_stops():
    """E-stop flag from the fake: the goal fails with 'e-stop' and the wheels stop."""
    rig = Rig()
    rig.goto('A')
    for _ in range(int(11.0 / STEP_S)):
        rig.step()
    rig.loop.esp.command('estop on', rig.loop.now)
    assert rig.run(10.0) == nav_core.Done(False, 'e-stop')
    rig.loop.run(1.0)
    last = rig.loop.msgs(codec.SensorData)[-1]
    before = last.left_ticks
    rig.loop.run(1.0)
    assert abs(rig.loop.msgs(codec.SensorData)[-1].left_ticks - before) <= 1
