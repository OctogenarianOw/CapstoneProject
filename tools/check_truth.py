#!/usr/bin/env python3
"""Run the terrain analysis and the per cell test on the ground truth itself.

No flight, no map, no simulation. This answers one question: does the world
test what we think it tests, that is, does the truth accept ramp_ok and
reject ramp_bad and rough under the retained thresholds. If it does not,
flying measures nothing.

Zone masks are eroded by one cell, because a cell on a zone border has an
analysis window straddling two zones and is legitimately rejected.

    python3 tools/check_truth.py
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'uav_core'))

from uav_core.slz_detection import classify_cells          # noqa: E402
from uav_core.terrain_analysis import analyse               # noqa: E402

MAX_SLOPE_DEG = 6.0
MAX_ROUGHNESS_M = 0.05
MAX_STEP_M = 0.24


def as_layers(out):
    """Accept either a dict or a four tuple from analyse()."""
    if isinstance(out, dict):
        return out
    slope, roughness, step, valid = out
    return {'slope': slope, 'roughness': roughness, 'step': step,
            'valid': valid}


def erode(mask):
    """Drop the one cell border of a mask, without scipy."""
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out &= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def report(title, elevation, zone, names, resolution):
    count = np.ones_like(elevation)
    layers = as_layers(analyse(elevation, count, resolution))
    ok = classify_cells(layers, MAX_SLOPE_DEG, MAX_ROUGHNESS_M, MAX_STEP_M)

    print(f'=== {title} ===')
    print(f'{"zone":10s} {"cells":>6s} {"slope":>8s} {"rough":>8s} '
          f'{"step":>8s} {"accepted":>9s}')
    for zid, name in enumerate(names):
        mask = erode(zone == zid) & layers['valid']
        if not mask.any():
            print(f'{name:10s} {0:6d}')
            continue
        print(f'{name:10s} {int(mask.sum()):6d} '
              f'{np.median(layers["slope"][mask]):8.2f} '
              f'{np.median(layers["roughness"][mask]):8.3f} '
              f'{np.median(layers["step"][mask]):8.3f} '
              f'{100.0 * ok[mask].mean():8.1f}%')
    print()


def main():
    data = np.load(ROOT / 'worlds' / 'validation_truth.npz')
    resolution = float(data['resolution'])
    names = [str(s) for s in data['zone_names']]
    zone = data['zone'].T                 # ENU [east, north] to NED [north, east]

    for key in ('truth_mean', 'truth_max'):
        report(key, -data[key].T, zone, names, resolution)


if __name__ == '__main__':
    main()