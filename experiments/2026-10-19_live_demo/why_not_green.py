#!/usr/bin/env python3
"""
Why is a truly flat cell not a landing candidate? Breakdown on one map.

Runs the manager's perception chain (world_model.build_world) on a map
snapshot and, for every cell whose analytic truth is the flat zone,
gives the first reason it is not a candidate, in pipeline order: never
seen, seen but not OBSERVED, analysis window incomplete, slope,
roughness, step, footprint touching a refused cell. Truth alignment as
in tools/validate_map.py.

    python3 why_not_green.py SNAPSHOT.npz [TRUTH.npz]
"""

from collections import Counter
import sys

import numpy as np
from uav_core.observation import OBSERVED, UNKNOWN
from uav_core.world_model import build_world, THRESHOLDS, WorldParams

snap = np.load(sys.argv[1])
truth_path = (sys.argv[2] if len(sys.argv) > 2 else
              '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz')
t = np.load(truth_path)
res = float(snap['resolution'])
origin = (float(snap['origin_north']), float(snap['origin_east']))
w = build_world(snap['elevation'], snap['variance'], snap['count'], res,
                origin, (0.0, 0.0), WorldParams())

zone = t['zone'].T
names = [str(n) for n in t['zone_names']]
n_t = zone.shape[0]
half = n_t * res / 2.0
i0 = int(round((-half + res / 2.0 - origin[0]) / res))
j0 = int(round((-half + res / 2.0 - origin[1]) / res))
sl = (slice(i0, i0 + n_t), slice(j0, j0 + n_t))

layers = {k: v[sl] for k, v in w.layers.items()}
state = w.state[sl]
cell_ok = w.landing['cell_ok'][sl]
cand = w.landing['candidate'][sl]


def reason(i, j):
    if state[i, j] == UNKNOWN:
        return 'never seen'
    if state[i, j] != OBSERVED:
        return 'seen, not OBSERVED'
    if not layers['valid'][i, j]:
        return 'analysis window not fully seen'
    if layers['slope'][i, j] > THRESHOLDS['max_slope_deg']:
        return 'slope > 6 deg'
    if layers['roughness'][i, j] > THRESHOLDS['max_roughness_m']:
        return 'roughness > 0.05 m'
    if layers['step'][i, j] > THRESHOLDS['max_step_m']:
        return 'step > 0.24 m'
    if not cell_ok[i, j]:
        return 'other gate'
    if not cand[i, j]:
        return 'footprint touches a refused cell'
    return 'CANDIDATE'


for zid, name in enumerate(names):
    sel = zone == zid
    counts = Counter(reason(i, j) for i, j in zip(*np.nonzero(sel)))
    total = int(sel.sum())
    print(f'== {name}: {total} cells')
    for r, n in counts.most_common():
        print(f'   {r:<36} {n:5d}  {100.0 * n / total:5.1f} %')
