"""
Path planning on the elevation map at cruise altitude (spec S12).

Role: say which cells the vehicle may fly over, how far every cell is
along a collision-free path, and return the path itself, reduced to the
few straight legs the executor will fly.

Traversability (invariant I3): a cell is blocked when its terrain stands
above altitude - vertical_margin, or when it was never seen (the
conservative policy of S12). Blocked cells are inflated by inflate_m,
1.25 m: 0.35 m half-span of the x500 plus 0.90 m, the largest tracking
error measured over 16 calibration flights on 19 Oct. A start cell that
falls inside the inflation is moved to the nearest free cell within
2 m, as the 49274 explorer does.

Distance field: the vectorised wavefront of the 49274 explorer, octile
path length on the 8-connected grid, with any number of sources. Seeded
from the vehicle it gives the outbound distance to every frontier in one
pass; seeded from every confirmed landing zone it gives, for every cell,
the distance to the nearest one, the return leg of C1, again in one
pass. Octile length exceeds the straight line by up to 8.2 %, an error
on the cautious side when it stands for the return distance.

Rejected: one A* search per pair (spec S12 says A*). C1 needs the
outbound and return distance of every candidate frontier each cycle; A*
would run once per pair for the same optimal grid paths.

Path: walked down the field from the goal, then simplified by line of
sight, each vertex kept only if the straight segment past it would cross
a blocked cell. Every vertex removed saves a leg, c_leg = 65 J and two
ramps in energy_model.py. Energy uses the simplified path, whose legs
are straight, so its length is Euclidean. Known limit: vertices come
from the raw grid path only, so around an obstacle it may keep one leg
more than an any-angle planner (Theta*) would, about 140 J; on open
ground, as in the validation world at 8 m, every path is one leg.

The rolling horizon and the 5 Hz path watchdog of S12 belong to the
manager, which owns replanning; this module stays a pure function.
"""

from dataclasses import dataclass
import math

import numpy as np

from uav_core.observation import UNKNOWN
from uav_core.slz_detection import dilate

# (d_row, d_col, step length in cells), 8-connected
NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
              (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]


@dataclass
class Plan:
    """A simplified path in cells and in metres, with its length."""

    cells: list          # [(i, j), ...] vertices kept after simplification
    waypoints_ne: list   # [(north, east), ...] cell centres in metres
    length_m: float      # sum of the straight legs


def blocked_mask(elevation, state, altitude_m, vertical_margin_m=2.0,
                 unknown_blocked=True):
    """
    Cells the vehicle must not overfly, before inflation.

    elevation is NED z (down), so terrain height is -elevation.
    """
    height = -np.where(np.isfinite(elevation), elevation, 0.0)
    blocked = height > altitude_m - vertical_margin_m
    if unknown_blocked:
        blocked |= state == UNKNOWN
    return blocked


def traversable_mask(elevation, state, resolution, altitude_m,
                     vertical_margin_m=2.0, inflate_m=1.25,
                     unknown_blocked=True):
    """Free cells, the blocked ones grown by the inflation radius."""
    blocked = blocked_mask(elevation, state, altitude_m, vertical_margin_m,
                           unknown_blocked)
    radius = int(math.ceil(inflate_m / resolution))
    return ~dilate(blocked, radius)


def nearest_traversable(traversable, ij, max_cells):
    """Return ij if free, else the closest free cell within max_cells."""
    h, w = traversable.shape
    i, j = ij
    if 0 <= i < h and 0 <= j < w and traversable[i, j]:
        return (i, j)
    rows, cols = np.nonzero(traversable)
    if rows.size == 0:
        return None
    d2 = (rows - i) ** 2 + (cols - j) ** 2
    k = int(np.argmin(d2))
    if d2[k] > max_cells * max_cells:
        return None
    return (int(rows[k]), int(cols[k]))


def distance_field(traversable, sources):
    """
    Octile path length in cells from the nearest source, inf elsewhere.

    sources: iterable of (i, j). Bellman-Ford sweeps vectorised with
    numpy, each relaxing every cell from its 8 neighbours, until nothing
    changes (49274 explorer, extended to several sources).
    """
    h, w = traversable.shape
    padded = np.full((h + 2, w + 2), np.inf, dtype=np.float64)
    dist = padded[1:-1, 1:-1]
    for i, j in sources:
        if traversable[i, j]:
            dist[i, j] = 0.0
    if not np.isfinite(dist).any():
        return np.full((h, w), np.inf)
    blocked = ~traversable
    new = np.empty((h, w))
    tmp = np.empty((h, w))
    for _ in range(h * w):
        new[:] = dist
        for di, dj, cost in NEIGHBOURS:
            np.add(padded[1 + di:1 + di + h, 1 + dj:1 + dj + w], cost,
                   out=tmp)
            np.minimum(new, tmp, out=new)
        new[blocked] = np.inf
        if np.array_equal(new, dist):
            break
        dist[:] = new
    return dist.copy()


def extract_path(dist, goal):
    """Follow the field downhill from goal to a source, return cells."""
    h, w = dist.shape
    if not np.isfinite(dist[goal]):
        return []
    path = [goal]
    i, j = goal
    for _ in range(h * w):
        if dist[i, j] == 0.0:
            break
        best = (i, j)
        for di, dj, _ in NEIGHBOURS:
            ni, nj = i + di, j + dj
            if 0 <= ni < h and 0 <= nj < w and dist[ni, nj] < dist[best]:
                best = (ni, nj)
        if best == (i, j):
            break
        i, j = best
        path.append(best)
    return path


def line_of_sight(traversable, a, b, step=0.25):
    """Tell whether the segment between two cell centres stays free."""
    (i0, j0), (i1, j1) = a, b
    n = max(1, int(math.ceil(math.hypot(i1 - i0, j1 - j0) / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    ii = np.floor(i0 + (i1 - i0) * t + 0.5).astype(int)
    jj = np.floor(j0 + (j1 - j0) * t + 0.5).astype(int)
    return bool(traversable[ii, jj].all())


def simplify(cells, traversable, step=0.25):
    """
    Keep a vertex only where the straight segment past it is blocked.

    Greedy: from each kept vertex, jump to the farthest later point in
    line of sight. All the lines of sight from a vertex are tested in
    one vectorised pass, each segment sampled exactly as line_of_sight
    samples it (its own count, shorter ones padded with their end
    point), so the result is that of testing them one by one. The one by
    one loop made one numpy call per point and up to quadratic calls per
    path: 2.0 s for the 225 paths of one cycle late in the 190 m flight
    of 19 Oct (experiments/2026-10-19_big_area).
    """
    if len(cells) <= 2:
        return list(cells)
    pts = np.asarray(cells, dtype=np.float64)
    out = [cells[0]]
    k = 0
    last = len(cells) - 1
    while k < last:
        delta = pts[k + 1:] - pts[k]
        n = np.maximum(1, np.ceil(np.hypot(delta[:, 0], delta[:, 1])
                                  / step)).astype(int)
        t = np.minimum(np.arange(int(n.max()) + 1)[None, :] / n[:, None],
                       1.0)
        ii = np.floor(pts[k, 0] + delta[:, 0, None] * t + 0.5).astype(int)
        jj = np.floor(pts[k, 1] + delta[:, 1, None] * t + 0.5).astype(int)
        visible = np.flatnonzero(traversable[ii, jj].all(axis=1))
        far = k + 1 + (int(visible[-1]) if visible.size else 0)
        out.append(cells[far])
        k = far
    return out


def cells_to_ne(cells, resolution, origin_ne):
    """Cell centres in NED metres."""
    return [(origin_ne[0] + (i + 0.5) * resolution,
             origin_ne[1] + (j + 0.5) * resolution) for i, j in cells]


def _make_plan(raw, traversable, resolution, origin_ne):
    cells = simplify(raw, traversable)
    ne = cells_to_ne(cells, resolution, origin_ne)
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(ne[:-1], ne[1:]))
    return Plan(cells=cells, waypoints_ne=ne, length_m=length)


def plan_path(traversable, dist_from_source, goal, resolution, origin_ne):
    """
    Simplified path from the source of a field to a goal cell, or None.

    The field must have been seeded at the start cell; the path is
    walked from the goal and reversed so it runs start to goal.
    """
    raw = extract_path(dist_from_source, goal)
    if not raw:
        return None
    return _make_plan(raw[::-1], traversable, resolution, origin_ne)


def descend_plan(traversable, dist_to_sources, start, resolution, origin_ne):
    """
    Simplified path from a start cell to the nearest source, or None.

    The field is seeded at the destinations, the landing zones for a
    return; walking it downhill from the start already runs in flight
    order, start to destination.
    """
    raw = extract_path(dist_to_sources, start)
    if not raw:
        return None
    return _make_plan(raw, traversable, resolution, origin_ne)
