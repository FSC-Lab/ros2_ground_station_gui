'''
MIT License

Copyright (c) 2026 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
"""Known-obstacle avoidance for setpoints the LLM features send (pure Python + numpy).

Obstacles are upright boxes in the mocap/ENU frame -- each either tracked live through an
OptiTrack rigid body on /vrpn_mocap/<name>/pose (the same frame as /uav_0/mocap: the
processor passes the VRPN pose through unchanged) or fixed in the file. The drone is a
point that must stay `margin_m` outside every box footprint unless it is `clearance_m`
above the box top (or below its bottom).

A route is: one straight segment if it is free; otherwise the altitude change is made
first, at the start, then a 2-D A* path on a grid at the target altitude, shortened by
line-of-sight. This is NOT sensing: an object not in the file is invisible to it.
"""
import heapq
import json
import math

import numpy as np

DEFAULT_MARGIN_M = 0.7       # T650: ~0.65 m motor-to-motor plus 15" props, plus slack
DEFAULT_CLEARANCE_M = 0.4    # drone centre over/under a box (leaves room for the landing gear)
# Routes are PLANNED this much further out than the margin, so that what is FLOWN stays
# outside it: following a sliding reference, the T650 sim still overshot an arrival by up to
# 0.12 m (2026-10-22: closest approach 0.64 m to a box with a 0.7 m margin, target 0.76 m).
DEFAULT_TRACKING_TOLERANCE_M = 0.15
DEFAULT_STALE_S = 1.0        # a tracked obstacle older than this blocks every route
GRID_M = 0.1
SAMPLE_M = 0.05
# Measured on the T650 sim with the baseline controller (2026-10-22): a 90 deg yaw setpoint
# step swings the drone 0.60 m sideways (0.75 m with a mid-turn hold) and a 0.75 m position
# step overshoots by ~0.25 m -- either eats most of the keep-out margin. 90 deg in 15 deg
# steps: 0.05 m; 30 deg: 0.14 m; 45 deg: 0.20 m. Hence RoutePath: nothing is ever stepped.


class Obstacle:
    """An upright box: centre (x, y, z), size (sx, sy, sz), yaw (deg) about z."""

    def __init__(self, name, center, size, yaw_deg=0.0):
        self.name = name
        self.center = np.asarray(center, dtype=float)
        self.size = np.asarray(size, dtype=float)
        self.yaw = math.radians(yaw_deg)
        self.bottom = self.center[2] - self.size[2] / 2.0
        self.top = self.center[2] + self.size[2] / 2.0

    def blocks_altitude(self, z, clearance):
        return self.bottom - clearance < z < self.top + clearance

    def footprint_distance(self, x, y):
        """Horizontal distance from (x, y) to the box footprint (0 inside it)."""
        dx, dy = x - self.center[0], y - self.center[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy   # into the box frame
        ex = max(abs(lx) - self.size[0] / 2.0, 0.0)
        ey = max(abs(ly) - self.size[1] / 2.0, 0.0)
        return math.hypot(ex, ey)

    def describe(self):
        return (f"{self.name} (centre {self.center[0]:.2f}, {self.center[1]:.2f}, "
                f"top {self.top:.2f} m)")


class ObstacleConfig:
    """The obstacles file: margins plus a list of tracked or fixed boxes."""

    def __init__(self, data, path=''):
        self.path = path
        self.margin = float(data.get('margin_m', DEFAULT_MARGIN_M))
        self.tolerance = float(data.get('tracking_tolerance_m', DEFAULT_TRACKING_TOLERANCE_M))
        self.clearance = float(data.get('clearance_m', DEFAULT_CLEARANCE_M))
        self.stale_s = float(data.get('stale_s', DEFAULT_STALE_S))
        self.entries = []
        for i, entry in enumerate(data.get('obstacles', [])):
            name = str(entry.get('name') or f'obstacle_{i}')
            size = [float(v) for v in entry['size']]
            if len(size) != 3 or min(size) <= 0:
                raise ValueError(f"{path}: obstacle {name}: size must be 3 positive numbers")
            topic = entry.get('topic')
            if topic:
                # Offset from the tracked pivot to the box centre, in the rigid body's
                # yaw frame. Default: pivot on the top centre (markers on the lid).
                offset = [float(v) for v in entry.get('center_offset', [0.0, 0.0, -size[2] / 2])]
                self.entries.append({'name': name, 'size': size, 'topic': str(topic),
                                     'center_offset': offset})
            else:
                position = [float(v) for v in entry['position']]
                self.entries.append({'name': name, 'size': size, 'topic': None,
                                     'position': position,
                                     'yaw_deg': float(entry.get('yaw_deg', 0.0))})

    @classmethod
    def load(cls, path):
        with open(path) as fh:
            return cls(json.load(fh), path)

    def topics(self):
        return [(e['name'], e['topic']) for e in self.entries if e['topic']]

    def resolve(self, tracked, now):
        """Obstacles at their current poses, plus a list of problems.

        tracked: name -> (position xyz, quaternion xyzw, monotonic receive time).
        A tracked obstacle with no recent pose is a problem: its position is unknown,
        so no route can be trusted.
        """
        obstacles, problems = [], []
        for e in self.entries:
            if e['topic'] is None:
                obstacles.append(Obstacle(e['name'], e['position'], e['size'], e['yaw_deg']))
                continue
            pose = tracked.get(e['name'])
            if pose is None:
                problems.append(f"obstacle {e['name']} has never been seen on {e['topic']}")
                continue
            position, q, stamp = pose
            if now - stamp > self.stale_s:
                problems.append(f"obstacle {e['name']} not tracked for {now - stamp:.1f} s")
                continue
            x, y, z, w = q
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            c, s = math.cos(yaw), math.sin(yaw)
            ox, oy, oz = e['center_offset']
            center = (position[0] + c * ox - s * oy, position[1] + s * ox + c * oy,
                      position[2] + oz)
            obstacles.append(Obstacle(e['name'], center, e['size'], math.degrees(yaw)))
        return obstacles, problems


class Planner:
    """margin: the closest the drone centre may FLY to a box footprint. tolerance: how far
    beyond that routes are PLANNED, for the controller's tracking error. Every check uses
    margin + tolerance; `margin` is kept for messages."""

    def __init__(self, obstacles, geofence, margin=DEFAULT_MARGIN_M,
                 clearance=DEFAULT_CLEARANCE_M, tolerance=DEFAULT_TRACKING_TOLERANCE_M):
        self.obstacles = obstacles
        self.fx, self.fy, self.fz = (float(v) for v in geofence)
        self.safety_margin = margin
        self.safety_clearance = clearance
        self.tolerance = tolerance
        # Planning uses both, plus the tolerance, horizontally AND vertically. With
        # tolerance=0 the same planner MONITORS a flight against the true limits, so the
        # tracking wobble the tolerance was planned for never triggers a re-plan (2026-10-22:
        # a leg ending exactly at a box's height + clearance re-planned every tick as the
        # altitude wobbled 1 cm across the line, and never arrived).
        self.margin = margin + tolerance
        self.clearance = clearance + tolerance

    def relaxed_margins(self, start):
        """Margins for a route that starts at `start`. If the drone is already inside an
        obstacle's margin (it drifted there), that obstacle's margin shrinks to the current
        distance: the route may not come any closer, but it may leave. Without this every
        route out of a drifted-in position, including the way out, would be refused."""
        x, y, z = start[:3]
        out = {}
        for ob in self.obstacles:
            if ob.blocks_altitude(z, self.clearance):
                d = ob.footprint_distance(x, y)
                if d < self.margin:
                    out[id(ob)] = max(0.0, d - 0.02)
        return out

    def blocker(self, p, margins=None, extra=0.0):
        """The obstacle the point is too close to (planning margin + extra), or None."""
        x, y, z = p
        for ob in self.obstacles:
            margin = self.margin if margins is None else margins.get(id(ob), self.margin)
            if ob.blocks_altitude(z, self.clearance) and ob.footprint_distance(x, y) < margin + extra:
                return ob
        return None

    def segment_blocker(self, a, b, margins=None):
        """The first obstacle the segment comes too close to, or None. Margins default to
        relaxed_margins(a): leaving a margin the segment starts in is allowed."""
        if margins is None:
            margins = self.relaxed_margins(a)
        a, b = np.asarray(a[:3], dtype=float), np.asarray(b[:3], dtype=float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / SAMPLE_M)))
        for k in range(n + 1):
            ob = self.blocker(a + (b - a) * k / n, margins)
            if ob is not None:
                return ob
        return None

    def route(self, a, b):
        """Waypoints (x, y, z) from a to b, excluding a; or (None, reason)."""
        a, b = tuple(float(v) for v in a[:3]), tuple(float(v) for v in b[:3])
        margins = self.relaxed_margins(a)
        ob = self.blocker(b, margins)
        if ob is not None:
            return None, (f"the target is within {self.margin:.2f} m of {ob.describe()} "
                          f"({self.safety_margin:.2f} m margin + {self.tolerance:.2f} m "
                          "tracking tolerance)")
        if self.segment_blocker(a, b, margins) is None:
            return [b], None
        # Change altitude in place first, then go around at the target altitude.
        corner = (a[0], a[1], b[2])
        if self.segment_blocker(a, corner, margins) is not None:
            corner = None
        if corner is not None:
            path = self._astar(corner, b, margins)
            if path is not None:
                return ([corner] if abs(corner[2] - a[2]) > 1e-6 else []) + path, None
        # Or go around at the current altitude and change altitude at the end.
        end = (b[0], b[1], a[2])
        if self.segment_blocker(end, b, margins) is None:
            path = self._astar(a, end, margins)
            if path is not None:
                return path + ([b] if abs(b[2] - a[2]) > 1e-6 else []), None
        blockers = {o.name for o in self.obstacles if o.blocks_altitude(b[2], self.clearance)}
        return None, ("no obstacle-free path at this altitude"
                      + (f" around {', '.join(sorted(blockers))}" if blockers else ""))

    def _astar(self, a, b, margins):
        z = a[2]
        nx = int(round(2 * self.fx / GRID_M)) + 1
        ny = int(round(2 * self.fy / GRID_M)) + 1

        def cell(p):
            return (min(nx - 1, max(0, int(round((p[0] + self.fx) / GRID_M)))),
                    min(ny - 1, max(0, int(round((p[1] + self.fy) / GRID_M)))))

        def point(c):
            return (-self.fx + c[0] * GRID_M, -self.fy + c[1] * GRID_M, z)

        free_cache = {}

        def free(c):
            # Cells are tested at their centres, so inflate by most of a cell: a step
            # between two free cells must not graze the margin either.
            if c not in free_cache:
                free_cache[c] = self.blocker(point(c), margins, extra=0.75 * GRID_M) is None
            return free_cache[c]

        start, goal = cell(a), cell(b)
        # Both ends already passed the exact check (route()); the grid's inflation must not
        # re-judge them, or a legal target near a box would have no path at all.
        free_cache[start] = True
        free_cache[goal] = True
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (-1, -1, math.sqrt(2))]
        open_set = [(0.0, 0.0, start)]
        came, cost = {start: None}, {start: 0.0}
        while open_set:
            _, g, c = heapq.heappop(open_set)
            if c == goal:
                break
            if g > cost.get(c, math.inf):
                continue
            for dx, dy, w in steps:
                n = (c[0] + dx, c[1] + dy)
                if not (0 <= n[0] < nx and 0 <= n[1] < ny) or not free(n):
                    continue
                ng = g + w
                if ng < cost.get(n, math.inf):
                    cost[n], came[n] = ng, c
                    h = math.hypot(goal[0] - n[0], goal[1] - n[1])
                    heapq.heappush(open_set, (ng + h, ng, n))
        if goal not in came:
            return None
        cells = []
        c = goal
        while c is not None:
            cells.append(c)
            c = came[c]
        points = [a] + [point(c) for c in reversed(cells[1:-1])] + [b]
        # Line-of-sight shortening: keep only the corners that are needed.
        out, i = [], 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and self.segment_blocker(points[i], points[j], margins) is not None:
                j -= 1
            out.append(points[j])
            i = j
        if any(self.segment_blocker(p, q, margins) is not None
               for p, q in zip([a] + out[:-1], out)):
            return None  # a grid corner case the shortening could not fix
        return [tuple(round(v, 3) for v in p) for p in out]


class RoutePath:
    """A planned route as a polyline, flown as a reference that slides along it.

    Setpoint STEPS overshoot on this airframe: the baseline controller overshoots a 0.75 m
    position step by ~0.25 m, and a 90 deg yaw step swings it 0.60 m sideways (measured on
    the T650 sim, 2026-10-22). So nothing jumps: the heading turns first, in place, at a
    capped rate; then the reference moves along the polyline at a capped speed, never more
    than a lead distance ahead of the drone (route follower in the ground station).
    """

    def __init__(self, start, waypoints, yaw):
        self.start_yaw = float(start[3]) if len(start) > 3 and start[3] is not None else float(yaw)
        self.yaw = float(yaw)
        self.turn = ((self.yaw - self.start_yaw + 180.0) % 360.0) - 180.0
        self.points = [np.asarray(start[:3], dtype=float)] + [np.asarray(w[:3], dtype=float)
                                                              for w in waypoints]
        self.cumulative = [0.0]
        for p, q in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + float(np.linalg.norm(q - p)))
        self.length = self.cumulative[-1]

    def heading_at(self, turned):
        """Heading after `turned` degrees of the turn (0..|turn|)."""
        frac = 0.0 if self.turn == 0 else min(1.0, turned / abs(self.turn))
        return ((self.start_yaw + self.turn * frac + 180.0) % 360.0) - 180.0

    def point_at(self, s):
        """Position at arc length s along the polyline."""
        s = min(max(s, 0.0), self.length)
        for i in range(1, len(self.points)):
            if s <= self.cumulative[i] or i == len(self.points) - 1:
                seg = self.cumulative[i] - self.cumulative[i - 1]
                t = 0.0 if seg <= 0 else (s - self.cumulative[i - 1]) / seg
                return tuple(float(v) for v in self.points[i - 1] + t * (self.points[i] - self.points[i - 1]))
        return tuple(float(v) for v in self.points[-1])
