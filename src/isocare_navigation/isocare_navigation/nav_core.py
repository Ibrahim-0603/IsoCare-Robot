"""ROS-free brain of the navigator node: drives the robot along a planned route.

The node in navigator.py only moves data between ROS and this module. Every decision lives
here, so it can be tested against a simulated robot in simulated time. Nothing here reads a
clock: the node passes `now` in.

How one goal works (target = a bed name or the dock):
    PLANNING   ask route_planner for a route from the last marker the robot reached
    DRIVE      for each marker on the route in turn: steer toward its position from the
               current pose (turn on the spot if more than ~20 degrees off, otherwise drive
               and steer proportionally), and move on once within arrive_radius_m of it
    BLOCKED    something is in front: wait (twice by default), then give up on this edge
    REPLANNING ask route_planner again with that edge excluded, from the marker just left
    APPROACH   (final marker only) creep forward until the standoff distance is reached
    FACE       turn on the spot to face the final marker
The goal ends with Done(success) or Done(False, reason). Reasons: 'blocked', 'no progress',
'no route', 'e-stop', 'link lost', 'cancelled'.
"""

from dataclasses import dataclass
import math

# States
IDLE = 'IDLE'
PLANNING = 'PLANNING'
DRIVE = 'DRIVE'
BLOCKED = 'BLOCKED'
REPLANNING = 'REPLANNING'
APPROACH = 'APPROACH'
FACE = 'FACE'


@dataclass
class NavConfig:
    """Tunable settings. The node fills these from ROS parameters."""

    max_speed_m_s: float = 0.3          # forward speed limit
    approach_speed_m_s: float = 0.1     # creeping speed in the final approach
    max_turn_rad_s: float = 0.5         # turning speed limit (turn on the spot)
    min_turn_rad_s: float = 0.15        # the motors stall below this
    turn_gain: float = 2.0              # turn-on-the-spot rate = gain x error, within the limits
    turn_threshold_deg: float = 20.0    # more than this off the target: stop and turn first
    turn_exit_deg: float = 6.0          # ... and keep turning until within this
    steer_gain: float = 1.5             # driving: turn rate = gain x error (0 = stop-turn-go)
    max_steer_rad_s: float = 0.5
    slow_zone_m: float = 1.0            # start slowing this far before the stopping radius
    min_speed_scale: float = 0.3
    arrive_radius_m: float = 0.5        # a marker counts as reached within this
    dock_name: str = 'dock'
    dock_standoff_m: float = 0.4        # the robot parks this far in front of the dock marker
    bed_standoff_m: float = 0.5         # and this far from a bed marker
    approach_margin_m: float = 0.3      # the final approach starts standoff + margin away
    heading_tol_deg: float = 10.0       # final facing tolerance
    blocked_stop_range_mm: float = 200.0
    blocked_clear_range_mm: float = 350.0
    blocked_wait_s: float = 10.0        # how long to wait for a blocked path to clear
    blocked_retries: int = 1            # extra waiting rounds on one edge before giving up
    progress_min_m: float = 0.2         # must get this much closer ...
    progress_timeout_s: float = 15.0    # ... within this time while driving, or 'no progress'
    final_timeout_s: float = 30.0       # the final approach and facing must finish in this time

    def __post_init__(self):
        """Reject settings that make no sense or could be unsafe."""
        positive = ('max_speed_m_s', 'approach_speed_m_s', 'max_turn_rad_s', 'min_turn_rad_s',
                    'turn_gain', 'turn_threshold_deg', 'turn_exit_deg', 'slow_zone_m',
                    'arrive_radius_m', 'dock_standoff_m', 'bed_standoff_m', 'heading_tol_deg',
                    'blocked_stop_range_mm', 'blocked_clear_range_mm', 'blocked_wait_s',
                    'progress_min_m', 'progress_timeout_s', 'final_timeout_s')
        for name in positive:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive number')
        if not (math.isfinite(self.steer_gain) and self.steer_gain >= 0.0):
            raise ValueError('steer_gain must be 0 or more (0 = stop-turn-go)')
        if not 0.0 < self.min_speed_scale <= 1.0:
            raise ValueError('min_speed_scale must be above 0 and at most 1')
        if self.blocked_retries < 0:
            raise ValueError('blocked_retries must be 0 or more')
        if self.min_turn_rad_s > self.max_turn_rad_s:
            raise ValueError('min_turn_rad_s must not exceed max_turn_rad_s')
        if self.turn_exit_deg >= self.turn_threshold_deg:
            raise ValueError('turn_exit_deg must be below turn_threshold_deg')
        if min(self.dock_standoff_m, self.bed_standoff_m) * 1000.0 <= self.blocked_stop_range_mm:
            raise ValueError('the standoffs must be farther than blocked_stop_range_mm, or the '
                             'robot would count its own parking spot as blocked')


@dataclass(frozen=True)
class Sample:
    """What the navigator sees each cycle. front_mm is None when there is no valid reading."""

    x_m: float
    y_m: float
    heading_rad: float
    front_mm: float = None
    estop: bool = False
    obst_stop: bool = False
    link_ok: bool = True


@dataclass(frozen=True)
class PlanResult:
    """The planner's answer: marker names, positions and facing, in route order."""

    ok: bool
    reason: str = ''
    names: tuple = ()
    xs: tuple = ()
    ys: tuple = ()
    headings: tuple = ()


# Commands: what the core asks the node to do
@dataclass(frozen=True)
class PlanRequest:
    """Call /plan_route (blocked is a tuple of (a, b) marker-name pairs to avoid)."""

    from_name: str
    to_name: str
    blocked: tuple = ()


@dataclass(frozen=True)
class CmdVel:
    """Publish this speed (m/s forward, rad/s counter-clockwise)."""

    linear: float
    angular: float


@dataclass(frozen=True)
class Done:
    """The goal is over. reason is '' on success."""

    success: bool
    reason: str = ''


@dataclass(frozen=True)
class Log:
    """A line for the node's log."""

    text: str


def wrap_angle(angle):
    """Return the angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """Return value limited to the range [low, high]."""
    return max(low, min(high, value))


def dock_pose(marker_x, marker_y, marker_heading, standoff_m):
    """Return (x, y, heading) of the robot parked in front of a marker, facing it."""
    return (marker_x + standoff_m * math.cos(marker_heading),
            marker_y + standoff_m * math.sin(marker_heading),
            wrap_angle(marker_heading + math.pi))


class NavCore:
    """The navigator state machine. See the module docstring."""

    def __init__(self, config=None, start_marker='dock'):
        """Start idle, standing at start_marker."""
        self.config = config if config is not None else NavConfig()
        self.state = IDLE
        self.last_marker = start_marker     # the last marker the robot reached
        self.status = ''                    # one line for the action feedback
        self.target = ''
        self._route = None
        self._idx = 0
        self._blocked_edges = []
        self._attempts = 0                  # waiting rounds used on the current edge
        self._turning = False
        self._blocked_since = None
        self._best_dist = math.inf
        self._progress_at = 0.0
        self._final_at = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, target, now):
        """Begin a goal. Returns the commands (a PlanRequest)."""
        if self.state != IDLE:
            return [Log(f'goal for {target} ignored: state is {self.state}')]
        self.target = target
        self._blocked_edges = []
        self._route = None
        self.state = PLANNING
        self.status = f'planning to {target}'
        return [Log(f'goal: {target}'), PlanRequest(self.last_marker, target)]

    def cancel(self, now):
        """Stop at once and end the goal as cancelled."""
        if self.state == IDLE:
            return []
        return self._fail('cancelled')

    def on_plan(self, result, now):
        """Take the planner's answer (for a PlanRequest from start() or a replan)."""
        if self.state not in (PLANNING, REPLANNING):
            return []
        if not result.ok:
            reason = 'no route' if self.state == PLANNING else 'blocked'
            return self._fail(reason, result.reason)
        self._route = result
        self._idx = 0
        self._attempts = 0
        self._turning = False
        self.state = DRIVE
        self._reset_progress(now)
        return [Log(f'route: {" -> ".join(result.names)}')]

    def update(self, sample, now):
        """Run one control cycle (call it about 20 times a second). Returns the commands."""
        if self.state == IDLE:
            return []
        if sample.estop:
            return self._fail('e-stop')
        if not sample.link_ok:
            return self._fail('link lost')
        if self.state in (PLANNING, REPLANNING):
            return [CmdVel(0.0, 0.0)]
        if self.state == BLOCKED:
            return self._blocked(sample, now)
        if self.state == DRIVE:
            return self._drive(sample, now)
        return self._final(sample, now)           # APPROACH or FACE

    # ------------------------------------------------------------------
    # Driving along the route
    # ------------------------------------------------------------------

    def _standoff(self):
        """Return how far in front of the final marker the robot stops."""
        return (self.config.dock_standoff_m if self.target == self.config.dock_name
                else self.config.bed_standoff_m)

    def _waypoint(self):
        """Return (name, x, y, is_final) of the marker being driven to."""
        route = self._route
        return (route.names[self._idx], route.xs[self._idx], route.ys[self._idx],
                self._idx == len(route.names) - 1)

    def _reset_progress(self, now):
        """Restart the no-progress timer."""
        self._best_dist = math.inf
        self._progress_at = now

    def _drive(self, sample, now):
        """Steer toward the current marker; handle arrival, blocking and no progress."""
        cfg = self.config
        commands = []
        while True:
            name, tx, ty, is_final = self._waypoint()
            dist = math.hypot(tx - sample.x_m, ty - sample.y_m)
            stop_radius = (self._standoff() + cfg.approach_margin_m if is_final
                           else cfg.arrive_radius_m)
            if dist > stop_radius:
                break
            if is_final:
                self.state = APPROACH
                self._final_at = now
                self._turning = False
                commands.append(Log(f'at {name}: final approach'))
                return commands + self._final(sample, now)
            self.last_marker = name
            self._idx += 1
            self._attempts = 0
            self._reset_progress(now)
            commands.append(Log(f'reached {name}'))

        error = wrap_angle(math.atan2(ty - sample.y_m, tx - sample.x_m) - sample.heading_rad)
        linear, angular = self._steer(error, dist, stop_radius)
        self.status = f'to {name}, {dist:.1f} m'

        if linear > 0.0 and self._is_blocked(sample):
            return commands + self._start_block(now)
        if linear <= 0.0:
            self._reset_progress(now)             # turning on the spot is not "no progress"
        elif dist < self._best_dist - cfg.progress_min_m:
            self._best_dist = dist
            self._progress_at = now
        elif now - self._progress_at > cfg.progress_timeout_s:
            return commands + self._fail('no progress')
        return commands + [CmdVel(linear, angular)]

    def _steer(self, error, dist, stop_radius):
        """Return (linear, angular) for a heading error and a distance to go."""
        cfg = self.config
        threshold = math.radians(cfg.turn_threshold_deg)
        if self._turning:
            if abs(error) < math.radians(cfg.turn_exit_deg):
                self._turning = False
        elif abs(error) > threshold:
            self._turning = True
        if self._turning:
            return 0.0, self._turn_rate(error)
        scale = clamp((dist - stop_radius) / cfg.slow_zone_m, cfg.min_speed_scale, 1.0)
        linear = cfg.max_speed_m_s * scale * (1.0 - 0.5 * abs(error) / threshold)
        angular = clamp(cfg.steer_gain * error, -cfg.max_steer_rad_s, cfg.max_steer_rad_s)
        return linear, angular

    def _turn_rate(self, error):
        """Return the turn-on-the-spot rate for a heading error (sign = direction)."""
        cfg = self.config
        rate = clamp(cfg.turn_gain * abs(error), cfg.min_turn_rad_s, cfg.max_turn_rad_s)
        return math.copysign(rate, error)

    # ------------------------------------------------------------------
    # Blocked path
    # ------------------------------------------------------------------

    def _is_blocked(self, sample):
        """Return True if something is close in front."""
        return sample.obst_stop or (sample.front_mm is not None
                                    and sample.front_mm <= self.config.blocked_stop_range_mm)

    def _is_clear(self, sample):
        """Return True if the way ahead is free again."""
        if sample.obst_stop:
            return False
        return sample.front_mm is None or sample.front_mm >= self.config.blocked_clear_range_mm

    def _start_block(self, now):
        """Stop and start a waiting round, unless this edge has used up its rounds."""
        if self._attempts > self.config.blocked_retries:
            return self._give_up_edge('blocked again after waiting')
        self.state = BLOCKED
        self._blocked_since = now
        self.status = 'blocked, waiting'
        return [Log('path blocked: waiting'), CmdVel(0.0, 0.0)]

    def _blocked(self, sample, now):
        """Hold still. A round ends when the path clears (resume) or the wait runs out.

        Every round used on this edge counts; after blocked_retries + 1 rounds the edge
        is given up (so with the default of 1: wait, wait once more, then plan around).
        """
        if self._is_clear(sample):
            self._attempts += 1
            self.state = DRIVE
            self._reset_progress(now)
            return [Log('path clear: resuming'), CmdVel(0.0, 0.0)]
        if now - self._blocked_since >= self.config.blocked_wait_s:
            self._attempts += 1
            if self._attempts > self.config.blocked_retries:
                return self._give_up_edge('still blocked after waiting')
            self._blocked_since = now
            return [Log('still blocked: waiting once more'), CmdVel(0.0, 0.0)]
        return [CmdVel(0.0, 0.0)]

    def _give_up_edge(self, why):
        """Mark the edge to the current marker blocked and plan around it."""
        if self._idx == 0:
            return self._fail('blocked', why)
        previous, current = self._route.names[self._idx - 1], self._route.names[self._idx]
        self._blocked_edges.append((previous, current))
        self.state = REPLANNING
        self.status = f'blocked at {previous}-{current}, replanning'
        return [Log(f'{why}: avoiding {previous}-{current}'), CmdVel(0.0, 0.0),
                PlanRequest(previous, self.target, tuple(self._blocked_edges))]

    # ------------------------------------------------------------------
    # Final approach: creep to the standoff, then face the marker
    # ------------------------------------------------------------------

    def _final(self, sample, now):
        """Creep to the standoff distance, then turn to face the final marker."""
        cfg = self.config
        name, tx, ty, _ = self._waypoint()
        if now - self._final_at > cfg.final_timeout_s:
            return self._fail('no progress', 'the final approach did not finish')
        tolerance = math.radians(cfg.heading_tol_deg)

        if self.state == APPROACH:
            dist = math.hypot(tx - sample.x_m, ty - sample.y_m)
            standoff = self._standoff()
            front_ok = sample.front_mm is not None
            reached = (front_ok and sample.front_mm / 1000.0 <= standoff) or dist <= standoff
            if reached:
                self.state = FACE
                return [Log(f'standoff reached at {name}: facing it'), CmdVel(0.0, 0.0)]
            error = wrap_angle(math.atan2(ty - sample.y_m, tx - sample.x_m) - sample.heading_rad)
            self.status = f'approaching {name}, {dist:.2f} m'
            if abs(error) > tolerance:
                return [CmdVel(0.0, self._turn_rate(error))]
            angular = clamp(cfg.steer_gain * error, -cfg.max_steer_rad_s, cfg.max_steer_rad_s)
            return [CmdVel(cfg.approach_speed_m_s, angular)]

        facing = self._route.headings[self._idx] + math.pi
        error = wrap_angle(facing - sample.heading_rad)
        self.status = f'facing {name}'
        if abs(error) <= tolerance:
            return self._succeed(name)
        return [CmdVel(0.0, self._turn_rate(error))]

    # ------------------------------------------------------------------
    # Ending a goal
    # ------------------------------------------------------------------

    def _succeed(self, name):
        """End the goal successfully at the final marker."""
        self.last_marker = name
        self.state = IDLE
        self.status = f'arrived at {name}'
        return [CmdVel(0.0, 0.0), Log(f'arrived at {name}'), Done(True)]

    def _fail(self, reason, detail=''):
        """End the goal as failed, stopped."""
        self.state = IDLE
        self.status = f'failed: {reason}'
        note = f' ({detail})' if detail else ''
        return [CmdVel(0.0, 0.0), Log(f'goal failed: {reason}{note}'), Done(False, reason)]
