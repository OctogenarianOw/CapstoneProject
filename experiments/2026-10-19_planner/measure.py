#!/usr/bin/env python3
"""
Planner on map v6: traversability at 8 m, field timing, one plan.

Times the outbound field (one source, the vehicle), the return field
(several sources, here four landing points) and a plan to the far
corner, on the 100 x 100 grid the manager will use at 1 Hz.

    python3 measure.py MAP.npz
"""

import sys
import time

import numpy as np
from uav_core.observation import classify, OBSERVED, UNKNOWN
from uav_core.planner import distance_field, plan_path, traversable_mask

m = np.load(sys.argv[1])
res = float(m['resolution'])
origin = (float(m['origin_x']), float(m['origin_y']))
state = classify(m['count'], m['variance'])
elev = m['elevation']


def cell(n, e):
    return (int((n - origin[0]) / res), int((e - origin[1]) / res))


free = traversable_mask(elev, state, res, altitude_m=8.0)
seen = state != UNKNOWN
print(f'cells seen {seen.sum()}, OBSERVED {(state == OBSERVED).sum()}, '
      f'traversable at 8 m {free.sum()} '
      f'({100 * free.sum() / seen.sum():.1f} % of seen)')
height = -np.where(np.isfinite(elev), elev, 0.0)
print(f'highest seen terrain {height[seen].max():.2f} m above EKF origin')

start = cell(0.0, 0.0)
for label, sources in (('outbound, 1 source', [start]),
                       ('return, 4 sources', [cell(-20, -20), cell(-20, 20),
                                              cell(20, -20), cell(20, 20)])):
    t0 = time.perf_counter()
    dist = distance_field(free, sources)
    ms = 1e3 * (time.perf_counter() - t0)
    print(f'{label}: {ms:.1f} ms, reachable {np.isfinite(dist).sum()}, '
          f'max {np.nanmax(np.where(np.isfinite(dist), dist, np.nan)):.1f}'
          ' cells')

dist = distance_field(free, [start])
goal = cell(25.0, 25.0)
t0 = time.perf_counter()
plan = plan_path(free, dist, goal, res, origin)
ms = 1e3 * (time.perf_counter() - t0)
print(f'plan origin -> N +25 E +25: {len(plan.cells) - 1} legs, '
      f'{plan.length_m:.2f} m, octile {dist[goal] * res:.2f} m, '
      f'{ms:.1f} ms')
