#!/usr/bin/env python3
"""
Where the tail of the flat ground of F5 comes from: the flat cells of
the reference map v6 whose slope reads over the 6 deg threshold.

F5 shows the flat noise floor at p95 4.34 deg but p99 21.66 deg. A
reader asks what that last per cent is. Measured here: each flat
interior cell over 6 deg (same alignment, analysis radius and one-cell
erosion as F5), its distance to the edge of every other truth zone,
to the border of the truth window, its observation count and the step
the map reads there.

Choice: distances to the truth zones rather than a visual inspection,
so the attribution is a number; the erosion kept at one cell, as in
F5, since the question is what F5 shows.

Result on v6 (27 Oct): 88 of 2484 cells (3.5 %), 81 within 3 m of
another zone's edge, 86 within 5 m, step read 0.41 m (median); the
truth under all of them is flat (slope 0, step 0). The truth zone
edges are steps, up to 0.81 m (3 deg ramp), 2.73 m (10 deg ramp),
0.78 m (rough) and 4.0 m (cube): the map smears them over about
1.5 m (P2), beyond the one cell the erosion removes. False refusals,
on the safe side.

    python3 f5_flat_tail.py        # ROS and the workspace sourced
"""

from pathlib import Path

import numpy as np
from scipy import ndimage

from uav_core.terrain_analysis import analyse

ROOT = Path(__file__).resolve().parents[2]
SLOPE_T = 6.0


def main():
    m = np.load(ROOT / 'data_survey_v6.npz')
    t = np.load(ROOT / 'worlds' / 'validation_truth.npz')
    res = float(m['resolution'])
    names = [str(n) for n in t['zone_names']]
    zone = t['zone'].T
    n = zone.shape[0]
    half = n * res / 2.0
    i0 = int(round((-half + res / 2.0 - float(m['origin_x'])) / res))
    j0 = int(round((-half + res / 2.0 - float(m['origin_y'])) / res))
    elev = m['elevation'][i0:i0 + n, j0:j0 + n]
    count = m['count'][i0:i0 + n, j0:j0 + n]
    lay = analyse(elev, count, res, radius=1)
    flat_code = names.index('flat')
    flat = ndimage.binary_erosion(zone == flat_code, np.ones((3, 3)))
    flat &= lay['valid']
    s = lay['slope']
    print(f'flat interior cells {flat.sum()}, p95 '
          f'{np.percentile(s[flat], 95):.2f} deg, p99 '
          f'{np.percentile(s[flat], 99):.2f} deg')
    high = flat & (s > SLOPE_T)
    print(f'over {SLOPE_T} deg: {high.sum()} '
          f'({100 * high.sum() / flat.sum():.1f} %)')
    for code, name in enumerate(names):
        if code == flat_code:
            continue
        d = ndimage.distance_transform_edt(zone != code) * res
        print(f'  nearest {name:<9}: median {np.median(d[high]):5.1f} m, '
              f'within 3 m {int((d[high] <= 3).sum())}')
    d_any = ndimage.distance_transform_edt(zone == flat_code) * res
    print(f'  to any other zone: within 2 m {int((d_any[high] <= 2).sum())}, '
          f'within 3 m {int((d_any[high] <= 3).sum())}, '
          f'within 5 m {int((d_any[high] <= 5).sum())}')
    ii, jj = np.nonzero(high)
    edge = np.minimum.reduce([ii, jj, n - 1 - ii, n - 1 - jj]) * res
    print(f'  to the truth window border: median {np.median(edge):.1f} m, '
          f'within 2 m {int((edge <= 2).sum())}')
    print(f'  observations: median {np.median(count[high]):.0f} against '
          f'{np.median(count[flat]):.0f} on all flat')
    print(f'  step read: median {np.median(lay["step"][high]):.2f} m')
    truth = -t['truth'].T
    tl = analyse(truth, np.ones_like(count), res, radius=1)
    print(f'  truth under them: slope max {np.nanmax(tl["slope"][high]):.2f} '
          f'deg, step max {np.nanmax(tl["step"][high]):.3f} m')
    # Height jump across each zone edge in the truth: the largest
    # difference between a zone cell and a 4-neighbour flat cell.
    up = t['truth'].T
    for code, name in enumerate(names):
        if code == flat_code:
            continue
        jump = 0.0
        for sh in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb_zone = np.roll(zone, sh, axis=(0, 1))
            nb_up = np.roll(up, sh, axis=(0, 1))
            sel = (zone == code) & (nb_zone == flat_code)
            if sel.any():
                jump = max(jump, float(np.abs(up[sel] - nb_up[sel]).max()))
        print(f'  truth edge jump {name:<9}: up to {jump:.2f} m')


if __name__ == '__main__':
    main()
