"""Offline checks of the obstacle planner: /usr/bin/python3 -m pytest -q -p no:anyio test/"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from Common.obstacle_planner import (Obstacle, ObstacleConfig, Planner,  # noqa: E402
                                     RoutePath)

FENCE = (5.0, 5.0, 2.5)
CABINET = Obstacle('pillar', (0.5, 1.1, 0.8), (0.3, 0.3, 1.6))


def clear(planner, start, waypoints):
    pts = [start] + list(waypoints)
    return all(planner.segment_blocker(p, q) is None for p, q in zip(pts, pts[1:]))


def test_free_line_is_one_segment():
    p = Planner([CABINET], FENCE)
    route, why = p.route((0, 0, 1), (0, -1.5, 1))
    assert route == [(0.0, -1.5, 1.0)] and why is None


def test_detour_around_cabinet():
    p = Planner([CABINET], FENCE)
    assert p.segment_blocker((0, 0, 1), (1, 2.2, 1)) is CABINET
    route, why = p.route((0, 0, 1), (1, 2.2, 1))
    assert route and why is None and len(route) >= 2
    assert clear(p, (0, 0, 1), route)
    assert route[-1] == (1.0, 2.2, 1.0)


def test_climb_over_low_box_is_straight():
    low = Obstacle('crate', (0.5, 1.1, 0.2), (0.6, 0.6, 0.4))  # top 0.4 m
    p = Planner([low], FENCE)
    route, _ = p.route((0, 0, 1), (1, 2.2, 1))                 # 1.0 > 0.4 + 0.3
    assert route == [(1.0, 2.2, 1.0)]


def test_target_too_close_is_refused():
    p = Planner([CABINET], FENCE)
    route, why = p.route((0, 0, 1), (0.5, 0.3, 1.0))
    assert route is None and 'within' in why


def test_tracked_obstacle_pose_and_staleness():
    cfg = ObstacleConfig({'obstacles': [{'name': 'cab', 'topic': '/vrpn_mocap/cab/pose',
                                         'size': [0.6, 0.6, 1.6]}]})
    # pivot on the lid: centre is 0.8 m below it
    obs, problems = cfg.resolve({'cab': ((0.55, 0.9, 1.6), (0, 0, 0, 1), 100.0)}, now=100.5)
    assert abs(cfg.tolerance - 0.15) < 1e-9
    assert not problems and abs(obs[0].center[2] - 0.8) < 1e-9
    obs, problems = cfg.resolve({'cab': ((0.55, 0.9, 1.6), (0, 0, 0, 1), 100.0)}, now=102.0)
    assert not obs and 'not tracked' in problems[0]
    obs, problems = cfg.resolve({}, now=0.0)
    assert 'never been seen' in problems[0]


def test_route_path_geometry():
    path = RoutePath((0, 0, 1, 170.0), [(1, 0, 1), (1, 2, 1)], -170.0)
    assert abs(path.length - 3.0) < 1e-9 and abs(path.turn - 20.0) < 1e-9  # the short way
    assert path.point_at(0.5) == (0.5, 0.0, 1.0) and path.point_at(2.0) == (1.0, 1.0, 1.0)
    assert path.point_at(9.9) == (1.0, 2.0, 1.0)
    assert abs(path.heading_at(10.0) - 180.0) < 1e-9 or abs(path.heading_at(10.0) + 180.0) < 1e-9
    assert abs(path.heading_at(20.0) + 170.0) < 1e-9


def test_drifted_inside_margin_can_leave_but_not_close_in():
    p = Planner([CABINET], FENCE)
    inside = (0.5, 0.55, 1.0)                     # 0.4 m from the pillar footprint
    route, why = p.route(inside, (0.5, -0.6, 1.0))  # straight away from it
    assert route == [(0.5, -0.6, 1.0)], why
    route, why = p.route(inside, (0.5, 0.45, 1.0))  # still inside, but farther: allowed
    assert route == [(0.5, 0.45, 1.0)], why
    route, why = p.route(inside, (0.5, 0.65, 1.0))  # closer than it already is: refused
    assert route is None
    assert p.segment_blocker(inside, (0.5, 0.65, 1.0)) is CABINET   # closer: blocked


def test_vertical_tolerance_and_monitoring():
    chair = Obstacle('chair', (-2.0, -2.5, 0.45), (0.5, 0.5, 0.9))   # top 0.9
    plan = Planner([chair], FENCE)                     # clearance 0.4 + tolerance 0.15
    over = (-2.0, -2.5, 1.2)
    assert plan.blocker(over) is chair                 # 0.3 m over the top: not planned
    assert plan.blocker((-2.0, -2.5, 1.5)) is None     # 0.6 m over: plannable
    monitor = Planner([chair], FENCE, tolerance=0.0)   # true limits while flying
    assert monitor.blocker((-2.0, -2.5, 1.32)) is None  # 0.42 m over: fine to be at
    assert monitor.blocker((-2.0, -2.5, 1.28)) is chair
    # a planned path at the planning limit never trips the monitor through a 1 cm wobble
    beside = (-2.0, -1.4, 1.0)                         # 0.85 m from the footprint: plannable
    assert plan.blocker(beside) is None
    assert monitor.segment_blocker((-2.0, -1.39, 1.01), beside) is None
