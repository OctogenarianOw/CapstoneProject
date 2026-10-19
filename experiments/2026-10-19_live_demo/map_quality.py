#!/usr/bin/env python3
"""
Map quality on the flat zone: exploration map against survey map.

For each map snapshot given, on the cells whose truth is the flat zone
and that were seen: elevation error against the truth (constant offset
removed, as tools/validate_map.py), observation count, fused variance,
and the terrain roughness and slope the analysis computes. Tells whether
the flat cells refused on the exploration map come from map error, and
how that error relates to how often a cell was observed.

    python3 map_quality.py SNAPSHOT.npz [SNAPSHOT.npz ...]
"""

from pathlib import Path
import sys

import numpy as np
from uav_core.terrain_analysis import analyse

TRUTH = '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz'
t = np.load(TRUTH)
truth = -t['truth'].T
zone = t['zone'].T
n_t = truth.shape[0]

for path in sys.argv[1:]:
    s = np.load(path)
    res = float(s['resolution'])
    on, oe = float(s['origin_north']), float(s['origin_east'])
    half = n_t * res / 2.0
    i0 = int(round((-half + res / 2.0 - on) / res))
    j0 = int(round((-half + res / 2.0 - oe) / res))
    sl = (slice(i0, i0 + n_t), slice(j0, j0 + n_t))
    elev, var, cnt = s['elevation'][sl], s['variance'][sl], s['count'][sl]
    lay = analyse(s['elevation'], s['count'], res)
    rough, slope = lay['roughness'][sl], lay['slope'][sl]
    flat = (zone == 0) & (cnt > 0)
    offset = np.median(elev[flat] - truth[flat])
    err = np.abs(elev - truth - offset)
    valid = flat & np.isfinite(rough)
    print(f'== {Path(path).parent.parent.parent.name}/{Path(path).name}')
    print(f'   flat cells seen {flat.sum()}, offset {offset:+.3f} m')
    print(f'   error  RMS {np.sqrt(np.mean(err[flat] ** 2)):.3f} m, '
          f'p95 {np.percentile(err[flat], 95):.3f} m')
    print(f'   count  median {np.median(cnt[flat]):.0f}, p10 '
          f'{np.percentile(cnt[flat], 10):.0f}')
    print(f'   roughness median {np.median(rough[valid]):.3f} m, share '
          f'> 0.05 m {100 * np.mean(rough[valid] > 0.05):.1f} %; slope '
          f'share > 6 deg {100 * np.mean(slope[valid] > 6.0):.1f} %')
    print(f'   {"count":>12} {"cells":>6} {"RMS err m":>9} '
          f'{"rough>0.05":>10} {"slope>6":>8}')
    for lo, hi in [(1, 50), (50, 100), (100, 200), (200, 500), (500, 1e9)]:
        sel = valid & (cnt >= lo) & (cnt < hi)
        if sel.sum() == 0:
            continue
        print(f'   {f"[{lo:g}, {hi:g})":>12} {sel.sum():6d} '
              f'{np.sqrt(np.mean(err[sel] ** 2)):9.3f} '
              f'{100 * np.mean(rough[sel] > 0.05):9.1f}% '
              f'{100 * np.mean(slope[sel] > 6.0):7.1f}%')
