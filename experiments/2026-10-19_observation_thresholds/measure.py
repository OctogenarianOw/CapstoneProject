#!/usr/bin/env python3
"""
Measure where a map cell becomes trustworthy, to set n_min and var_max (S6).

The observation state layer splits cells into UNKNOWN, SPARSE and
OBSERVED. Its two thresholds must come from a measurement (spec S6).
The measurement: elevation error against the analytic truth, per bin of
observation count and per bin of fused variance, on the flat zone only.
Flat ground isolates the noise of the estimate from the two-surface
bias of P2 at structure edges, which no amount of observation removes.

Alignment is that of tools/validate_map.py: truth transposed and negated
into NED, constant offset removed with the median over flat cells.

    python3 measure.py MAP.npz [TRUTH.npz]
"""

import sys

import numpy as np

MAP = sys.argv[1]
TRUTH = (sys.argv[2] if len(sys.argv) > 2
         else '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz')

m = np.load(MAP)
t = np.load(TRUTH)
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])
truth = -t['truth'].T
zone = t['zone'].T
names = [str(n) for n in t['zone_names']]
n_t = truth.shape[0]
half = n_t * res / 2.0
i0 = int(round((-half + res / 2.0 - ox) / res))
j0 = int(round((-half + res / 2.0 - oy) / res))
elev = m['elevation'][i0:i0 + n_t, j0:j0 + n_t]
var = m['variance'][i0:i0 + n_t, j0:j0 + n_t]
cnt = m['count'][i0:i0 + n_t, j0:j0 + n_t]

flat = zone == 0
known = cnt > 0
offset = np.median(elev[flat & known] - truth[flat & known])
err = elev - truth - offset
print(f'map {MAP}, zones {names}, flat zone = {names[0]}')
print(f'offset removed {offset:+.3f} m, flat cells {flat.sum()}, '
      f'observed {int((flat & known).sum())}')


def table(values, edges, label):
    print(f'\n{label:>18} {"cells":>6} {"rms m":>7} {"p95 m":>7} '
          f'{"max m":>7}')
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = flat & known & (values >= lo) & (values < hi)
        if not sel.any():
            continue
        e = np.abs(err[sel])
        print(f'{f"[{lo:g}, {hi:g})":>18} {sel.sum():>6} '
              f'{np.sqrt(np.mean(e ** 2)):>7.3f} '
              f'{np.percentile(e, 95):>7.3f} {e.max():>7.3f}')


table(cnt, [1, 2, 3, 5, 10, 20, 50, 100, 200, 500, 1000, 1e9],
      'count')
table(var, [0, 5e-4, 1e-3, 2e-3, 5e-3, 1e-2, 2e-2, 5e-2, 1e-1, np.inf],
      'variance m2')
print(f'\ncount quantiles over observed flat cells: '
      f'{np.percentile(cnt[flat & known], [1, 5, 25, 50]).tolist()}')
print('variance against count, median per count bin:')
for lo, hi in [(1, 2), (2, 3), (3, 5), (5, 10), (10, 20), (20, 50),
               (50, 100), (100, 1e9)]:
    sel = flat & known & (cnt >= lo) & (cnt < hi)
    if sel.any():
        print(f'  count [{lo:g}, {hi:g}): median variance '
              f'{np.median(var[sel]):.2e}')
