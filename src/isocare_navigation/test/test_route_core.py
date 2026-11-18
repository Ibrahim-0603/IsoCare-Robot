"""Tests for isocare_navigation.route_core. Plain pytest, no ROS needed."""

import itertools
import math
import os
from pathlib import Path
import random
import subprocess
import sys

try:
    from isocare_navigation import route_core as rc
except ImportError:  # running from the repo without sourcing the workspace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isocare_navigation import route_core as rc
import pytest

DEMO_FILE = Path(__file__).resolve().parents[2] / 'isocare_bringup' / 'config' / 'markers.yaml'


def square_map():
    """Return a map shaped like a square with a diagonal-free ring: a, b, c, d (side 2 m)."""
    return rc.MarkerMap.from_dict({
        'markers': [
            {'id': 0, 'name': 'a', 'x': 0.0, 'y': 0.0},
            {'id': 1, 'name': 'b', 'x': 2.0, 'y': 0.0},
            {'id': 2, 'name': 'c', 'x': 2.0, 'y': 2.0},
            {'id': 3, 'name': 'd', 'x': 0.0, 'y': 2.0}],
        'edges': [['a', 'b'], ['b', 'c'], ['c', 'd'], ['d', 'a']]})


def demo_data():
    """Return a small valid table as plain data (to be broken on purpose by tests)."""
    return {
        'markers': [{'id': 0, 'name': 'dock', 'x': 0.0, 'y': 0.0},
                    {'id': 1, 'name': 'j', 'x': 3.0, 'y': 0.0},
                    {'id': 2, 'name': 'A', 'x': 3.0, 'y': 2.0}],
        'edges': [['dock', 'j'], ['j', 'A']]}


# --------------------------------------------------------------------------
# Planning
# --------------------------------------------------------------------------

def test_straight_route_distances_and_headings_come_from_positions():
    """Plan dock to A on the demo data: 3 m along +x, then 2 m along +y."""
    route = rc.MarkerMap.from_dict(demo_data()).plan('dock', 'A')
    assert route.names == ('dock', 'j', 'A')
    assert route.marker_ids == (0, 1, 2)
    assert [leg.distance_m for leg in route.legs] == pytest.approx([3.0, 2.0])
    assert [leg.heading_rad for leg in route.legs] == pytest.approx([0.0, math.pi / 2])
    assert route.total_m == pytest.approx(5.0)


def test_route_carries_the_marker_positions_and_facing():
    """The route also hands over each marker's position and facing, in route order."""
    graph = rc.MarkerMap.from_dict(broken(
        lambda d: d['markers'][2].update(heading_deg=-90.0)))
    route = graph.plan('dock', 'A')
    assert [m.name for m in route.markers] == ['dock', 'j', 'A']
    assert (route.markers[2].x_m, route.markers[2].y_m) == (3.0, 2.0)
    assert route.markers[2].heading_rad == pytest.approx(-math.pi / 2)
    assert graph.plan('j', 'j').markers[0].name == 'j'


def test_reverse_route_has_opposite_headings():
    """Driving back is the same legs reversed, heading turned by 180 degrees."""
    graph = rc.MarkerMap.from_dict(demo_data())
    there, back = graph.plan('dock', 'A'), graph.plan('A', 'dock')
    assert back.names == tuple(reversed(there.names))
    assert back.total_m == pytest.approx(there.total_m)
    assert back.legs[0].heading_rad == pytest.approx(-math.pi / 2)    # straight down
    assert abs(back.legs[1].heading_rad) == pytest.approx(math.pi)    # back along -x


def test_totals_are_floats_even_for_a_route_with_no_legs():
    """Return floats everywhere: rclpy aborts the process on an int where a float is declared."""
    graph = rc.MarkerMap.from_dict(demo_data())
    assert isinstance(graph.plan('j', 'j').total_m, float)
    assert isinstance(graph.plan('dock', 'A').total_m, float)


def test_route_to_self_is_one_marker_and_no_legs():
    """Already there: a route with a single marker."""
    route = rc.MarkerMap.from_dict(demo_data()).plan('j', 'j')
    assert route.names == ('j',) and route.legs == () and route.total_m == 0.0


def test_shorter_route_wins_even_with_more_legs():
    """Dijkstra compares total distance, not the number of legs."""
    graph = rc.MarkerMap.from_dict({
        'markers': [{'id': 0, 'name': 's', 'x': 0.0, 'y': 0.0},
                    {'id': 1, 'name': 'm1', 'x': 1.0, 'y': 0.1},
                    {'id': 2, 'name': 'm2', 'x': 2.0, 'y': 0.0},
                    {'id': 3, 'name': 'g', 'x': 3.0, 'y': 0.1},
                    {'id': 4, 'name': 'far', 'x': 1.5, 'y': 5.0}],
        'edges': [['s', 'm1'], ['m1', 'm2'], ['m2', 'g'],       # about 3.0 m in three legs
                  ['s', 'far'], ['far', 'g']]})                  # about 10.5 m in two legs
    assert graph.plan('s', 'g').names == ('s', 'm1', 'm2', 'g')


def test_blocked_edge_makes_the_planner_go_around():
    """With one side of the ring blocked, the route takes the other way."""
    graph = square_map()
    assert graph.plan('a', 'c').total_m == pytest.approx(4.0)
    around = graph.plan('a', 'c', blocked=[('a', 'b')])
    assert around.names == ('a', 'd', 'c')
    assert graph.plan('a', 'c', blocked=[('b', 'a')]).names == ('a', 'd', 'c')   # either order


def test_blocking_every_way_through_is_an_error_naming_the_goal():
    """No way around: RouteError says there is no route."""
    with pytest.raises(rc.RouteError, match='no route from "a" to "c"'):
        square_map().plan('a', 'c', blocked=[('a', 'b'), ('a', 'd')])


def test_blocked_edge_that_does_not_exist_is_harmless():
    """Blocking a pair that is not an edge changes nothing."""
    assert square_map().plan('a', 'c', blocked=[('a', 'c')]).total_m == pytest.approx(4.0)


def test_unknown_names_are_errors():
    """A typo in a name is reported, never silently ignored."""
    graph = square_map()
    with pytest.raises(rc.RouteError, match='unknown marker "z"'):
        graph.plan('z', 'a')
    with pytest.raises(rc.RouteError, match='unknown marker "z"'):
        graph.plan('a', 'z')
    with pytest.raises(rc.RouteError, match='blocked edge'):
        graph.plan('a', 'c', blocked=[('a', 'z')])


def test_equal_length_routes_give_the_same_answer_in_every_process():
    """Ties are broken by name order, so the answer does not depend on Python's hash seed."""
    code = (
        'import sys; sys.path.insert(0, sys.argv[1])\n'
        'from isocare_navigation import route_core as rc\n'
        'g = rc.MarkerMap.from_dict({"markers": [{"id": 0, "name": "a", "x": 0.0, "y": 0.0},'
        '{"id": 1, "name": "b", "x": 2.0, "y": 0.0}, {"id": 2, "name": "c", "x": 2.0, "y": 2.0},'
        '{"id": 3, "name": "d", "x": 0.0, "y": 2.0}],'
        '"edges": [["a","b"],["b","c"],["c","d"],["d","a"]]})\n'
        'print(g.plan("a", "c").names)\n')
    package_root = str(Path(__file__).resolve().parents[1])
    answers = set()
    for seed in range(8):
        out = subprocess.run([sys.executable, '-c', code, package_root], check=True,
                             capture_output=True, text=True,
                             env={**os.environ, 'PYTHONHASHSEED': str(seed)})
        answers.add(out.stdout.strip())
    assert len(answers) == 1, answers


def test_random_maps_match_a_brute_force_shortest_path():
    """On 200 random connected maps, the planner's distance equals Floyd-Warshall's."""
    rng = random.Random(7)
    for _ in range(200):
        count = rng.randint(2, 8)
        names = [f'm{i}' for i in range(count)]
        points = {n: (rng.uniform(0, 10), rng.uniform(0, 10)) for n in names}
        edges = {(names[i], names[rng.randrange(i)]) for i in range(1, count)}   # a tree
        for pair in itertools.combinations(names, 2):                          # extra links
            if rng.random() < 0.3:
                edges.add(pair)
        edges = {tuple(sorted(e)) for e in edges}
        graph = rc.MarkerMap.from_dict({
            'markers': [{'id': i, 'name': n, 'x': points[n][0], 'y': points[n][1]}
                        for i, n in enumerate(names)],
            'edges': [list(e) for e in edges]})
        dist = {(a, b): (0.0 if a == b else math.inf) for a in names for b in names}
        for a, b in edges:
            d = math.dist(points[a], points[b])
            dist[(a, b)] = dist[(b, a)] = d
        for k in names:
            for a in names:
                for b in names:
                    dist[(a, b)] = min(dist[(a, b)], dist[(a, k)] + dist[(k, b)])
        for a, b in itertools.product(names, names):
            route = graph.plan(a, b)
            assert route.total_m == pytest.approx(dist[(a, b)])
            assert route.names[0] == a and route.names[-1] == b
            for first, second in zip(route.names, route.names[1:]):
                assert second in graph.edges[first]                  # only real edges are used


# --------------------------------------------------------------------------
# Validation of the table
# --------------------------------------------------------------------------

def broken(mutate):
    """Return the demo data after mutate(data), to be passed to from_dict."""
    data = demo_data()
    mutate(data)
    return data


@pytest.mark.parametrize('mutate,message', [
    (lambda d: d['markers'].append({'id': 0, 'name': 'x', 'x': 9.0, 'y': 9.0}),
     'two markers have id 0'),
    (lambda d: d['markers'].append({'id': 9, 'name': 'A', 'x': 9.0, 'y': 9.0}),
     'two markers are called "A"'),
    (lambda d: d['edges'].append(['j', 'nowhere']), '"nowhere", which is not a marker'),
    (lambda d: d['edges'].append(['j', 'j']), 'to itself'),
    (lambda d: d['edges'].append(['j', 'dock']), 'listed twice'),
    (lambda d: d['markers'].append({'id': 7, 'name': 'island', 'x': 50.0, 'y': 50.0}), 'island'),
    (lambda d: d['markers'][1].update(x=0.0, y=0.0), 'same position'),
    (lambda d: d['markers'][0].pop('x'), 'missing "x"'),
    (lambda d: d['markers'][0].update(x='three'), 'finite number'),
    (lambda d: d['markers'][0].update(x=float('nan')), 'finite number'),
    (lambda d: d['markers'][0].update(id=-1), 'id must be a whole number'),
    (lambda d: d['markers'][0].update(id=1.5), 'id must be a whole number'),
    (lambda d: d['markers'][0].update(name=''), 'name must be'),
    (lambda d: d['markers'][0].update(heading_deg='up'), 'heading_deg'),
    (lambda d: d.update(edges=[]), '"edges"'),
    (lambda d: d.update(markers=d['markers'][:1]), 'at least two'),
    (lambda d: d['edges'].append('dock-j'), 'must be a pair'),
])
def test_bad_tables_are_rejected_with_a_clear_message(mutate, message):
    """Every usual editing mistake raises ValueError that says what and where."""
    with pytest.raises(ValueError, match=message):
        rc.MarkerMap.from_dict(broken(mutate))


def test_table_must_be_a_mapping():
    """A file that parses to a list or nothing is refused."""
    for bad in (None, [], 'text'):
        with pytest.raises(ValueError, match='mapping'):
            rc.MarkerMap.from_dict(bad)


def test_heading_deg_defaults_to_zero_and_is_converted_to_radians():
    """The facing direction is optional and stored in radians."""
    graph = rc.MarkerMap.from_dict(broken(
        lambda d: d['markers'][2].update(heading_deg=-90.0)))
    assert graph.markers['dock'].heading_rad == 0.0
    assert graph.markers['A'].heading_rad == pytest.approx(-math.pi / 2)


# --------------------------------------------------------------------------
# The real files
# --------------------------------------------------------------------------

def test_demo_marker_table_loads_and_has_the_names_the_manager_uses():
    """The placeholder map in isocare_bringup is valid and contains dock and the beds."""
    graph = rc.MarkerMap.load_file(DEMO_FILE)
    assert {'dock', 'A', 'B'} <= set(graph.markers)
    # via A is 8.54 m, via junction2 is 9.0 m
    assert graph.plan('dock', 'B').names == ('dock', 'junction1', 'A', 'B')
    around = graph.plan('dock', 'B', blocked=[('A', 'B')])
    assert around.names == ('dock', 'junction1', 'junction2', 'B')


def test_load_file_errors_name_the_file(tmp_path):
    """Missing file, bad YAML and bad content all raise ValueError mentioning the path."""
    with pytest.raises(ValueError, match='cannot read marker table'):
        rc.MarkerMap.load_file(tmp_path / 'nothing.yaml')
    bad_yaml = tmp_path / 'bad.yaml'
    bad_yaml.write_text('markers: [unclosed')
    with pytest.raises(ValueError, match='bad.yaml'):
        rc.MarkerMap.load_file(bad_yaml)
    bad_content = tmp_path / 'content.yaml'
    bad_content.write_text('markers: []\nedges: []\n')
    with pytest.raises(ValueError, match='content.yaml'):
        rc.MarkerMap.load_file(bad_content)
