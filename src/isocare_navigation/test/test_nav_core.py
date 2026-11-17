"""Tests for isocare_navigation.nav_core against a simulated robot. No ROS needed.

The simulated robot is an ideal unicycle with the ESP32's forward-only obstacle stop. The real
route_planner core answers the plan requests, using the demo marker table or a small custom map.
"""

import math
from pathlib import Path
import sys

try:
    from isocare_navigation import nav_core as nc
    from isocare_navigation import route_core as rc
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_navigation import nav_core as nc
    from isocare_navigation import route_core as rc
import pytest

DEMO_FILE = Path(__file__).resolve().parents[2] / 'isocare_bringup' / 'config' / 'markers.yaml'
DT = 0.05


def chain_map():
    """Return a map with exactly one way: dock - j - A (no alternative routes)."""
    return rc.MarkerMap.from_dict({
        'markers': [{'id': 0, 'name': 'dock', 'x': 0.0, 'y': 0.0, 'heading_deg': 0.0},
                    {'id': 1, 'name': 'j', 'x': 3.0, 'y': 0.0, 'heading_deg': 180.0},
                    {'id': 2, 'name': 'A', 'x': 3.0, 'y': 2.0, 'heading_deg': -90.0}],
        'edges': [['dock', 'j'], ['j', 'A']]})


class Sim:
    """An ideal robot driven by a NavCore, answering its plan requests like the node does."""

    def __init__(self, graph=None, start_marker='dock', pose=None, obstacles=(), slip=1.0,
                 **config):
        """Park the robot at the dock (or at pose) and create the core."""
        self.graph = graph if graph is not None else rc.MarkerMap.load_file(DEMO_FILE)
        self.core = nc.NavCore(nc.NavConfig(**config), start_marker)
        if pose is None:
            dock = self.graph.markers['dock']
            standoff = self.core.config.dock_standoff_m
            pose = nc.dock_pose(dock.x_m, dock.y_m, dock.heading_rad, standoff)
        self.x, self.y, self.heading = pose
        self.obstacles = list(obstacles)      # (x, y, until_time or None)
        self.slip = slip                      # 0.0 = the wheels spin but the robot goes nowhere
        self.now = 0.0
        self.estop = False
        self.link_ok = True
        self.done = None
        self.cmd = (0.0, 0.0)
        self.history = []                     # (time, linear, angular) of every command
        self.log = []
        self.plans = []                       # every PlanRequest

    # -- the world ---------------------------------------------------------

    def front_mm(self):
        """Return the front range in mm (None if nothing within 3 m in a 40 degree cone)."""
        best = None
        for ox, oy, until in self.obstacles:
            if until is not None and self.now >= until:
                continue
            dx, dy = ox - self.x, oy - self.y
            if abs(nc.wrap_angle(math.atan2(dy, dx) - self.heading)) < math.radians(20):
                dist = math.hypot(dx, dy)
                if dist < 3.0 and (best is None or dist < best):
                    best = dist
        return None if best is None else best * 1000.0

    def sample(self):
        """Return what the navigator sees right now."""
        front = self.front_mm()
        return nc.Sample(self.x, self.y, self.heading, front, self.estop,
                         front is not None and front < 150.0, self.link_ok)

    def physics(self):
        """Move the robot for one step with the ESP32's rules (no forward motion when close)."""
        linear, angular = self.cmd
        front = self.front_mm()
        if front is not None and front < 150.0:
            linear = min(linear, 0.0)
        linear *= self.slip
        mid = self.heading + angular * DT / 2.0
        self.x += linear * math.cos(mid) * DT
        self.y += linear * math.sin(mid) * DT
        self.heading = nc.wrap_angle(self.heading + angular * DT)

    # -- the node's job ------------------------------------------------------

    def apply(self, commands):
        """Carry out the core's commands like the node does."""
        for command in commands:
            if isinstance(command, nc.CmdVel):
                self.cmd = (command.linear, command.angular)
                self.history.append((self.now, command.linear, command.angular))
            elif isinstance(command, nc.Done):
                self.done = command
            elif isinstance(command, nc.Log):
                self.log.append(command.text)
            elif isinstance(command, nc.PlanRequest):
                self.plans.append(command)
                self.apply(self.core.on_plan(self.plan(command), self.now))

    def plan(self, request):
        """Answer a PlanRequest with the real planner core."""
        try:
            route = self.graph.plan(request.from_name, request.to_name, request.blocked)
        except rc.RouteError as error:
            return nc.PlanResult(False, str(error))
        return nc.PlanResult(True, '', route.names, tuple(m.x_m for m in route.markers),
                             tuple(m.y_m for m in route.markers),
                             tuple(m.heading_rad for m in route.markers))

    def goto(self, target):
        """Give the navigator a goal."""
        self.apply(self.core.start(target, self.now))

    def step(self):
        """Run one control cycle and move the robot."""
        self.apply(self.core.update(self.sample(), self.now))
        self.physics()
        self.now = round(self.now + DT, 6)

    def run_until_state(self, state, limit_s=120.0):
        """Run until the core is in the given state. Fails if that takes longer than limit_s."""
        end = self.now + limit_s
        while self.core.state != state:
            assert self.now < end and self.done is None, f'never reached {state}'
            self.step()

    def run(self, limit_s=300.0):
        """Run until the goal ends. Fails if it takes longer than limit_s."""
        end = self.now + limit_s
        while self.done is None:
            assert self.now < end, f'no result after {limit_s} s, state {self.core.state}'
            self.step()
        return self.done

    def range_to(self, name):
        """Return the distance from the robot to a marker (the standoff is measured this way)."""
        marker = self.graph.markers[name]
        return math.hypot(marker.x_m - self.x, marker.y_m - self.y)

    def at(self, name, standoff):
        """Return how far the robot is from where it should stand in front of a marker."""
        marker = self.graph.markers[name]
        ex, ey, eh = nc.dock_pose(marker.x_m, marker.y_m, marker.heading_rad, standoff)
        return (math.hypot(self.x - ex, self.y - ey),
                abs(nc.wrap_angle(self.heading - eh)))


# --------------------------------------------------------------------------
# Driving to places
# --------------------------------------------------------------------------

def test_dock_to_bed_a_arrives_at_the_standoff_facing_the_marker():
    """The whole trip over two legs: the demo map's dock to bed A."""
    sim = Sim()
    sim.goto('A')
    assert sim.run() == nc.Done(True)
    off, turn = sim.at('A', 0.5)
    assert off < 0.2 and turn < math.radians(10)         # within 20 cm, facing within 10 degrees
    assert sim.core.last_marker == 'A'
    assert sim.cmd == (0.0, 0.0)


def test_bed_to_dock_returns_to_the_dock_pose():
    """Returning parks the robot at the dock pose again (same pose startup uses)."""
    sim = Sim()
    sim.goto('A')
    sim.run()
    sim.done = None
    sim.goto('dock')
    assert sim.run() == nc.Done(True)
    off, turn = sim.at('dock', 0.4)
    assert off < 0.2 and turn < math.radians(10)
    assert sim.core.last_marker == 'dock'
    assert sim.plans[-1].from_name == 'A'                # planned from the last marker reached


def test_two_beds_in_a_row_use_the_last_marker_as_the_start():
    """Go dock to A, then A to B: the second plan starts at A."""
    sim = Sim()
    sim.goto('A')
    sim.run()
    sim.done = None
    sim.goto('B')
    assert sim.run() == nc.Done(True)
    assert sim.plans[-1] == nc.PlanRequest('A', 'B')
    assert sim.at('B', 0.5)[1] < math.radians(10)


def test_speed_limits_are_never_exceeded():
    """Forward speed stays under 0.3 m/s (0.1 in the final approach), turning under 0.5 rad/s."""
    sim = Sim()
    sim.goto('B')
    sim.run()
    assert max(abs(v) for _, v, _ in sim.history) <= 0.3 + 1e-9
    assert max(abs(w) for _, _, w in sim.history) <= 0.5 + 1e-9
    assert all(v >= 0.0 for _, v, _ in sim.history)       # it never reverses


def test_it_turns_on_the_spot_before_driving_when_facing_the_wrong_way():
    """Starting 180 degrees off: no forward motion until the heading error is small."""
    sim = Sim()
    sim.goto('A')
    first_forward = None
    while sim.done is None and first_forward is None:
        sim.step()
        if sim.cmd[0] > 0.0:
            first_forward = sim.now
            heading_error = abs(nc.wrap_angle(
                math.atan2(0.0 - sim.y, 3.0 - sim.x) - sim.heading))
            assert heading_error < math.radians(20.0)
    assert first_forward is not None


def test_turn_direction_does_not_chatter_at_the_wrap_boundary():
    """Target almost exactly behind: the robot picks a direction and keeps it."""
    sim = Sim(pose=(0.4, 0.0, math.radians(-0.5)))        # faces +x, junction is ahead...
    sim.graph = chain_map()
    sim.core.last_marker = 'dock'
    sim.x, sim.y, sim.heading = 2.0, 0.0, math.pi - 0.001  # ... now point it straight back
    sim.core.last_marker = 'j'
    sim.goto('A')
    signs = []
    while sim.done is None and sim.now < 6.0:
        sim.step()
        if sim.cmd[0] == 0.0 and abs(sim.cmd[1]) > 0:
            signs.append(math.copysign(1, sim.cmd[1]))
    assert len(set(signs)) == 1                           # never flips between left and right


def test_stop_turn_go_still_gets_there_with_steering_gain_zero():
    """The fallback: no steering while driving, only turning on the spot when over 20 degrees."""
    sim = Sim(steer_gain=0.0)
    sim.goto('B')
    assert sim.run() == nc.Done(True)
    assert abs(sim.range_to('B') - 0.5) < 0.15            # at the standoff from the marker


def test_final_approach_stops_at_the_front_range_standoff():
    """A wall right at the marker: the creep stops when the front range reads the standoff."""
    graph = chain_map()
    wall = [(3.0, 2.0, None)]
    sim = Sim(graph=graph, obstacles=wall, start_marker='j',
              pose=(3.0, 0.2, math.pi / 2))
    sim.goto('A')
    assert sim.run() == nc.Done(True)
    assert 0.45 < 2.0 - sim.y < 0.62                      # stopped about 0.5 m from the wall


def test_goal_to_the_marker_the_robot_is_already_at_still_does_the_final_approach():
    """Start == goal: a route of one marker, then the approach and facing."""
    sim = Sim()
    sim.goto('dock')
    assert sim.run() == nc.Done(True)
    assert sim.at('dock', 0.4)[1] < math.radians(10)


# --------------------------------------------------------------------------
# Blocked paths
# --------------------------------------------------------------------------

def test_a_block_that_clears_within_the_wait_is_resumed_and_the_goal_succeeds():
    """A person stands in the corridor until t = 17 s (the robot arrives about t = 12 s)."""
    sim = Sim(obstacles=[(2.0, 0.0, 17.0)])
    sim.goto('A')
    assert sim.run() == nc.Done(True)
    assert any('waiting' in line for line in sim.log)
    assert any('resuming' in line for line in sim.log)
    assert len(sim.plans) == 1                            # never had to replan


def test_a_permanent_block_waits_then_plans_around_it():
    """An obstacle on the corridor to bed A: after the waits it goes around via the other route."""
    sim = Sim(obstacles=[(2.74, 1.0, None)])              # on the junction1 - A leg
    sim.goto('B')                                         # dock -> junction1 -> A -> B is shortest
    assert sim.run() == nc.Done(True)
    assert len(sim.plans) == 2
    assert sim.plans[1].from_name == 'junction1'
    assert sim.plans[1].blocked == (('junction1', 'A'),)
    assert abs(sim.range_to('B') - 0.5) < 0.15
    assert sim.at('B', 0.5)[1] < math.radians(10)


def test_a_permanent_block_waits_twice_by_default_before_giving_up():
    """Rounds: 10 s wait, one more 10 s wait (the retry), then replan: at least 20 s stopped."""
    sim = Sim(obstacles=[(2.74, 1.0, None)])
    sim.goto('B')
    while len(sim.plans) < 2:
        sim.step()
        assert sim.now < 120.0
    stopped = [t for t, v, w in sim.history if v == 0.0 and w == 0.0 and 5.0 < t < sim.now]
    assert sim.now - stopped[0] >= 19.0


def test_no_alternative_route_fails_as_blocked():
    """On a chain map there is no way around: the goal ends with reason 'blocked'."""
    sim = Sim(graph=chain_map(), obstacles=[(2.74, 1.0, None)])
    sim.goto('A')
    result = sim.run()
    assert result == nc.Done(False, 'blocked')
    assert sim.cmd == (0.0, 0.0)


def test_blocked_a_third_time_gives_up_the_edge_at_once():
    """Two rounds used (both cleared), a third block is not waited for: replan right away."""
    sim = Sim(obstacles=[(1.7, 0.0, 17.0), (2.2, 0.0, 28.0), (2.4, 0.0, None)])
    sim.goto('A')
    sim.run()
    assert 'blocked again after waiting' in ' '.join(sim.log)
    assert sim.plans[1].blocked == (('dock', 'junction1'),)       # no other way out of the dock
    assert sim.done == nc.Done(False, 'blocked')


def test_blocked_waiting_robot_never_commands_forward_motion():
    """While waiting in front of an obstacle every command is zero."""
    sim = Sim(obstacles=[(2.0, 0.0, 30.0)])
    sim.goto('A')
    while sim.done is None and sim.now < 40.0 and sim.core.state != nc.BLOCKED:
        sim.step()
    assert sim.core.state == nc.BLOCKED
    start = len(sim.history)
    for _ in range(40):
        sim.step()
        if sim.core.state != nc.BLOCKED:
            break
    assert all(v == 0.0 and w == 0.0 for _, v, w in sim.history[start:start + 30])


def test_turning_in_place_is_not_treated_as_blocked():
    """Obstacle right in front while the robot must turn away: it turns (and is not 'blocked')."""
    sim = Sim(graph=chain_map(), start_marker='j', pose=(3.0, 0.0, 0.0),
              obstacles=[(3.1, 0.0, None)])               # 10 cm ahead, target is behind
    sim.goto('dock')
    for _ in range(40):
        sim.step()
    assert abs(sim.heading) > math.radians(30)            # it has started turning away
    assert sim.core.state != nc.BLOCKED


# --------------------------------------------------------------------------
# Failures and stopping
# --------------------------------------------------------------------------

def test_no_progress_when_the_wheels_spin_but_the_robot_does_not_move():
    """Slip factor 0: after about 15 s of driving without getting closer, fail."""
    sim = Sim(slip=0.0)
    sim.goto('A')
    result = sim.run()
    assert result == nc.Done(False, 'no progress')
    assert 14.0 < sim.now < 25.0


def test_turning_on_the_spot_for_a_long_time_is_not_no_progress():
    """A slow turn must not trip the progress timer: it only counts forward driving."""
    sim = Sim(max_turn_rad_s=0.15, min_turn_rad_s=0.15, progress_timeout_s=5.0)
    sim.goto('A')                                         # needs a 180 degree turn: about 21 s
    while sim.done is None and sim.now < 22.0:
        sim.step()
    assert sim.done is None or sim.done.reason != 'no progress'


def test_unknown_target_fails_as_no_route():
    """The planner says unknown marker: the goal fails with 'no route'."""
    sim = Sim()
    sim.goto('nowhere')
    assert sim.run() == nc.Done(False, 'no route')


def test_estop_stops_and_fails_at_once():
    """An E-stop during the drive ends the goal with 'e-stop' and a zero command."""
    sim = Sim()
    sim.goto('A')
    for _ in range(100):
        sim.step()
    sim.estop = True
    assert sim.run(5.0) == nc.Done(False, 'e-stop')
    assert sim.cmd == (0.0, 0.0)


def test_lost_link_stops_and_fails():
    """A dead serial link ends the goal with 'link lost'."""
    sim = Sim()
    sim.goto('A')
    for _ in range(100):
        sim.step()
    sim.link_ok = False
    assert sim.run(5.0) == nc.Done(False, 'link lost')


def test_cancel_stops_at_once_in_every_state():
    """A cancel during planning, driving, blocking and the final approach ends the goal."""
    for stage in ('planning', 'driving', 'blocked', 'approach'):
        sim = Sim(obstacles=[(2.0, 0.0, None)] if stage == 'blocked' else ())
        sim.goto('A')
        if stage == 'planning':
            sim.core.state = nc.PLANNING
        elif stage == 'driving':
            for _ in range(60):
                sim.step()
        elif stage == 'blocked':
            sim.run_until_state(nc.BLOCKED)
        else:
            sim.run_until_state(nc.APPROACH)
        sim.apply(sim.core.cancel(sim.now))
        assert sim.done == nc.Done(False, 'cancelled'), stage
        assert sim.cmd == (0.0, 0.0), stage
        assert sim.core.state == nc.IDLE, stage


def test_cancel_when_idle_does_nothing():
    """Nothing to cancel, nothing happens."""
    assert nc.NavCore().cancel(0.0) == []


def test_a_second_goal_while_busy_is_ignored():
    """Only one goal at a time."""
    sim = Sim()
    sim.goto('A')
    out = sim.core.start('B', sim.now)
    assert all(isinstance(c, nc.Log) for c in out)
    assert sim.core.target == 'A'


def test_update_while_idle_commands_nothing():
    """An idle navigator publishes nothing, so the bridge's stale-command stop applies."""
    assert nc.NavCore().update(nc.Sample(0.0, 0.0, 0.0), 0.0) == []


def test_plan_answers_that_do_not_belong_are_ignored():
    """A late planner answer when no plan was requested changes nothing."""
    core = nc.NavCore()
    assert core.on_plan(nc.PlanResult(True, '', ('dock',), (0.0,), (0.0,), (0.0,)), 0.0) == []
    assert core.state == nc.IDLE


# --------------------------------------------------------------------------
# Helpers and config
# --------------------------------------------------------------------------

def test_dock_pose_is_in_front_of_the_marker_facing_it():
    """Marker at (1, 2) facing +y: the robot stands 0.4 m up from it, looking down."""
    x, y, heading = nc.dock_pose(1.0, 2.0, math.pi / 2, 0.4)
    assert (x, y) == pytest.approx((1.0, 2.4))
    assert heading == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('kwargs', [
    {'max_speed_m_s': 0.0}, {'max_turn_rad_s': -1.0}, {'turn_gain': math.nan},
    {'steer_gain': -0.1}, {'min_speed_scale': 0.0}, {'min_speed_scale': 1.5},
    {'blocked_retries': -1}, {'min_turn_rad_s': 0.9}, {'turn_exit_deg': 30.0},
    {'bed_standoff_m': 0.1}, {'dock_standoff_m': 0.15}, {'blocked_wait_s': 0.0}])
def test_bad_config_is_rejected(kwargs):
    """Settings that make no sense or could park the robot inside its own stop zone raise."""
    with pytest.raises(ValueError):
        nc.NavConfig(**kwargs)


def test_status_text_names_the_current_target():
    """The feedback line says where the robot is heading and how far it is."""
    sim = Sim()
    sim.goto('A')
    for _ in range(200):
        sim.step()
    assert sim.core.status and ('junction1' in sim.core.status or 'A' in sim.core.status)


def test_final_approach_follows_the_range_sensor_not_only_the_odometry():
    """A wall 0.3 m in front of the marker position: the robot stops 0.5 m from the WALL."""
    graph = chain_map()
    sim = Sim(graph=graph, obstacles=[(3.0, 1.7, None)], start_marker='j',
              pose=(3.0, 0.2, math.pi / 2))
    sim.goto('A')
    assert sim.run() == nc.Done(True)
    # the wall is at y = 1.7; odometry alone would drive on to 0.5 m from the marker (y = 1.5)
    assert 1.7 - sim.y == pytest.approx(0.5, abs=0.12)


def test_a_goal_that_ends_midway_leaves_the_last_reached_marker_for_the_next_plan():
    """Cancelled after passing junction1: the next goal is planned from junction1, not the dock."""
    sim = Sim()
    sim.goto('A')
    while 'reached junction1' not in sim.log:
        sim.step()
        assert sim.now < 60.0
    sim.apply(sim.core.cancel(sim.now))
    assert sim.core.last_marker == 'junction1'
    sim.done = None
    sim.goto('dock')
    assert sim.plans[-1] == nc.PlanRequest('junction1', 'dock')
