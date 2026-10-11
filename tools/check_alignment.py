"""Test whether the map-to-truth comparison is misaligned.

Shifts the truth grid by a few cells in each direction and reports the
RMS error for each shift. If some shift other than (0, 0) is clearly
better, the comparison in validate_map.py is off by that much. If (0, 0)
wins, the alignment is right and the residual is real measurement error.

Run against the ramps, where a one-cell shift on a slope creates a large
vertical error and the effect is unmistakable.
"""

import sys
import numpy as np

MAP_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/elevation_map.npz'
TRUTH_PATH = (sys.argv[2] if len(sys.argv) > 2
              else '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz')

SHIFT = 3        # test shifts from -3 to +3 cells in each axis

m = np.load(MAP_PATH)
elev = m['elevation']
count = m['count']
res = float(m['resolution'])
ox, oy = float(m['origin_x']), float(m['origin_y'])

t = np.load(TRUTH_PATH)
truth_ned = -t['truth'].T
zone_ned = t['zone'].T

n_t = truth_ned.shape[0]
half_t = n_t * res / 2.0

i0 = int(round((-half_t + res / 2.0 - ox) / res))
j0 = int(round((-half_t + res / 2.0 - oy) / res))

print(f'base alignment: map index ({i0}, {j0})')
print(f'map origin ({ox:+.1f}, {oy:+.1f}), truth half-size {half_t:.1f} m')
print()


def rms_for_shift(di, dj, zone_codes):
    """RMS error when the map window is moved by (di, dj) cells."""
    a, b = i0 + di, j0 + dj
    if a < 0 or b < 0 or a + n_t > elev.shape[0] or b + n_t > elev.shape[1]:
        return np.nan

    sub_elev = elev[a:a + n_t, b:b + n_t]
    sub_count = count[a:a + n_t, b:b + n_t]
    known = sub_count > 0

    # Recalibrate the constant offset for every shift, otherwise the
    # offset from the base alignment would bias the comparison.
    ref = known & (zone_ned == 0)
    if ref.sum() < 50:
        return np.nan
    offset = np.median(sub_elev[ref] - truth_ned[ref])

    sel = known & np.isin(zone_ned, zone_codes)
    if sel.sum() == 0:
        return np.nan

    e = sub_elev[sel] - truth_ned[sel] - offset
    return float(np.sqrt(np.mean(e ** 2)))


for label, codes in [('ramps only', [1, 2]),
                     ('all terrain', [0, 1, 2, 3, 4])]:
    print(f'--- {label} ---')
    header = '       ' + ''.join(f'{d:>8}' for d in
                                  range(-SHIFT, SHIFT + 1))
    print(f'  dj -> {header[7:]}')
    best = (np.inf, None)
    for di in range(-SHIFT, SHIFT + 1):
        row = f'di {di:+d} '
        for dj in range(-SHIFT, SHIFT + 1):
            v = rms_for_shift(di, dj, codes)
            row += f'{v:>8.3f}' if np.isfinite(v) else f'{"-":>8}'
            if np.isfinite(v) and v < best[0]:
                best = (v, (di, dj))
        print(row)
    print(f'  best: RMS {best[0]:.3f} at shift {best[1]}')
    print()