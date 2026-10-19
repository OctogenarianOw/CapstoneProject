#!/usr/bin/env python3
"""
C1 and C2 on map v6 cut to the take-off footprint, offline.

The situation the explorer faces after its climb: 18 m seen around
take-off, the take-off point as the only (provisional) zone. Prints the
candidate table under several budgets and alphas, and the time of one
evaluation.

    python3 measure.py MAP.npz
"""

import sys
import time
from types import SimpleNamespace

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.mission_policy import evaluate, PolicyParams, return_field
from uav_core.world_model import build_world, WorldParams

m = np.load(sys.argv[1])
res = float(m['resolution'])
origin = (float(m['origin_x']), float(m['origin_y']))
ii, jj = np.indices(m['count'].shape)
seen = np.hypot(origin[0] + (ii + 0.5) * res,
                origin[1] + (jj + 0.5) * res) <= 18.0
count = np.where(seen, m['count'], 0)
var = np.where(seen, m['variance'], np.inf)
elev = np.where(seen, m['elevation'], np.nan)
world = build_world(elev, var, count, res, origin, (0.0, 0.0),
                    WorldParams())
zones = [SimpleNamespace(zone_id=1, touchdown_ne=(0.0, 0.0))]
rf = return_field(world.traversable, zones, res, origin)

t0 = time.perf_counter()
d = evaluate(world, rf, MODEL, 2e4, PolicyParams(alpha=0.5), res, origin)
print(f'{len(d.candidates)} candidates, evaluation '
      f'{1e3 * (time.perf_counter() - t0):.1f} ms')
print(f'{"viewpoint N, E":>16} {"legs":>4} {"E_go J":>7} {"E_ret J":>7} '
      f'{"E_tot J":>7} {"gain":>5}')
for c in d.candidates:
    print(f'{c.frontier.viewpoint_ne[0]:+7.1f} {c.frontier.viewpoint_ne[1]:+7.1f}'
          f' {len(c.plan_go.cells) - 1:4d} {c.e_go:7.0f} {c.e_return:7.0f}'
          f' {c.e_total:7.0f} {c.frontier.gain:5d}')

for b in (2e4, 7000.0, 6500.0):
    for alpha in (0.0, 0.5, 1.0):
        d = evaluate(world, rf, MODEL, b, PolicyParams(alpha=alpha), res,
                     origin)
        n_adm = sum(c.admissible for c in d.candidates)
        best = d.best
        what = (f'best N {best.frontier.viewpoint_ne[0]:+5.1f} E '
                f'{best.frontier.viewpoint_ne[1]:+5.1f} G {best.g:.2f} '
                f'M {best.m:.2f} U {best.u:.2f}' if best else d.reason)
        print(f'B {b:7.0f} J alpha {alpha:.1f}: {n_adm:2d} admissible, '
              f'{what}')
