"""ROS-free brain of the route_planner node: the marker graph and Dijkstra.

The map comes from the shared marker table (a YAML file, see isocare_bringup/config/markers.yaml).
Each marker is a point; each listed edge is a free path between two markers. The distance and
the heading of a leg are computed from the marker positions, so the positions are the only
numbers anyone has to measure.
"""

from dataclasses import dataclass
import heapq
import math


class RouteError(Exception):
    """A route could not be planned (unknown marker, no way through)."""


@dataclass(frozen=True)
class Marker:
    """One marker: where it is and which way it faces (metres, radians)."""

    marker_id: int
    name: str
    x_m: float
    y_m: float
    heading_rad: float


@dataclass(frozen=True)
class Leg:
    """One straight leg between two connected markers."""

    from_name: str
    to_name: str
    distance_m: float
    heading_rad: float      # direction of travel in the map frame, 0 = +x, counter-clockwise


@dataclass(frozen=True)
class Route:
    """The planned route: the markers in order and the legs between them."""

    names: tuple
    marker_ids: tuple
    legs: tuple
    total_m: float
    markers: tuple = ()     # the Marker of each name, same order (positions and facing)


def _number(value, what):
    """Return value as a finite float or raise ValueError."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{what} must be a finite number, got {value!r}')
    return float(value)


class MarkerMap:
    """The marker graph. Build it with from_dict() or load_file(), then call plan()."""

    MIN_EDGE_M = 0.05       # two connected markers closer than this are surely a typo

    def __init__(self, markers, edges):
        """Store already validated markers (dict name -> Marker) and edges (name -> set)."""
        self.markers = markers
        self.edges = edges

    @classmethod
    def from_dict(cls, data):
        """Build and validate a map from the parsed YAML; ValueError says what is wrong."""
        if not isinstance(data, dict):
            raise ValueError('the marker table must be a mapping with "markers" and "edges"')
        raw_markers = data.get('markers')
        raw_edges = data.get('edges')
        if not isinstance(raw_markers, list) or len(raw_markers) < 2:
            raise ValueError('"markers" must be a list with at least two markers')
        if not isinstance(raw_edges, list) or not raw_edges:
            raise ValueError('"edges" must be a non-empty list of [name, name] pairs')

        markers, ids = {}, {}
        for index, entry in enumerate(raw_markers):
            where = f'markers[{index}]'
            if not isinstance(entry, dict):
                raise ValueError(f'{where} must be a mapping with id, name, x, y')
            for key in ('id', 'name', 'x', 'y'):
                if key not in entry:
                    raise ValueError(f'{where} is missing "{key}"')
            marker_id, name = entry['id'], entry['name']
            if isinstance(marker_id, bool) or not isinstance(marker_id, int) or marker_id < 0:
                raise ValueError(f'{where}: id must be a whole number, 0 or more')
            if not isinstance(name, str) or not name.strip():
                raise ValueError(f'{where}: name must be a non-empty text')
            if name in markers:
                raise ValueError(f'two markers are called "{name}"')
            if marker_id in ids:
                raise ValueError(
                    f'two markers have id {marker_id} ("{ids[marker_id]}" and "{name}")')
            heading_deg = _number(entry.get('heading_deg', 0.0), f'{where} heading_deg')
            markers[name] = Marker(marker_id, name, _number(entry['x'], f'{where} x'),
                                   _number(entry['y'], f'{where} y'),
                                   math.radians(heading_deg))
            ids[marker_id] = name

        edges = {name: set() for name in markers}
        for index, pair in enumerate(raw_edges):
            where = f'edges[{index}]'
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                raise ValueError(f'{where} must be a pair [name, name]')
            first, second = pair
            for name in pair:
                if name not in markers:
                    raise ValueError(f'{where} names "{name}", which is not a marker')
            if first == second:
                raise ValueError(f'{where} connects "{first}" to itself')
            if second in edges[first]:
                raise ValueError(f'{where}: "{first}" and "{second}" are listed twice')
            a, b = markers[first], markers[second]
            if math.hypot(b.x_m - a.x_m, b.y_m - a.y_m) < cls.MIN_EDGE_M:
                raise ValueError(f'{where}: "{first}" and "{second}" are at the same position')
            edges[first].add(second)
            edges[second].add(first)

        start = next(iter(markers))
        seen, stack = {start}, [start]
        while stack:
            for neighbour in edges[stack.pop()]:
                if neighbour not in seen:
                    seen.add(neighbour)
                    stack.append(neighbour)
        lonely = sorted(set(markers) - seen)
        if lonely:
            raise ValueError(
                f'these markers cannot be reached from "{start}": {", ".join(lonely)}')
        return cls(markers, edges)

    @classmethod
    def load_file(cls, path):
        """Read a YAML marker table; ValueError (with the file name) if it is not usable."""
        import yaml
        try:
            with open(path, encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(f'cannot read marker table {path}: {error}') from error
        try:
            return cls.from_dict(data)
        except ValueError as error:
            raise ValueError(f'{path}: {error}') from error

    def edge_count(self):
        """Return the number of edges."""
        return sum(len(neighbours) for neighbours in self.edges.values()) // 2

    def leg(self, from_name, to_name):
        """Return the leg from one marker to another (distance and heading from positions)."""
        a, b = self.markers[from_name], self.markers[to_name]
        dx, dy = b.x_m - a.x_m, b.y_m - a.y_m
        return Leg(from_name, to_name, math.hypot(dx, dy), math.atan2(dy, dx))

    def plan(self, start, goal, blocked=()):
        """Return the shortest Route from start to goal, avoiding the blocked (a, b) pairs.

        Raises RouteError for an unknown marker or when blocked edges leave no way through.
        """
        for name in (start, goal):
            if name not in self.markers:
                raise RouteError(f'unknown marker "{name}"')
        banned = set()
        for first, second in blocked:
            for name in (first, second):
                if name not in self.markers:
                    raise RouteError(f'unknown marker "{name}" in a blocked edge')
            banned.add(frozenset((first, second)))

        best = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]
        done = set()
        while queue:
            cost, name = heapq.heappop(queue)
            if name in done:
                continue
            done.add(name)
            if name == goal:
                break
            for neighbour in sorted(self.edges[name]):          # sorted: same answer every time
                if frozenset((name, neighbour)) in banned:
                    continue
                new_cost = cost + self.leg(name, neighbour).distance_m
                if new_cost < best.get(neighbour, math.inf):
                    best[neighbour] = new_cost
                    previous[neighbour] = name
                    heapq.heappush(queue, (new_cost, neighbour))
        if goal not in done:
            raise RouteError(f'no route from "{start}" to "{goal}" with those edges blocked'
                             if banned else f'no route from "{start}" to "{goal}"')

        names = [goal]
        while names[-1] != start:
            names.append(previous[names[-1]])
        names.reverse()
        legs = tuple(self.leg(a, b) for a, b in zip(names, names[1:]))
        return Route(tuple(names), tuple(self.markers[n].marker_id for n in names), legs,
                     sum(leg.distance_m for leg in legs),
                     tuple(self.markers[n] for n in names))
