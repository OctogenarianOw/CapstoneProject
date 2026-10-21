#!/usr/bin/env python3
"""Generate the second pit calibration world, wider pits, and its truth.

Role: the first pit calibration (experiments/2026-10-21_pit_calibration,
world pits) found 1 m pits filled on the map at every range: passed
overhead, pits 0.5 to 2.0 m deep read 0.13 to 0.21 m, below the 0.24 m
step threshold, so no depth met the criterion and W3 was not built on
them. The map fills a 1 m pit with the plate-top points around it that
fall into its cell through the horizontal error of the pose and the
attitude, as it fills the depressions of W1's rough zone. A pit wide
enough keeps a core that no neighbour point reaches; the question left
for W3 is whether such a pit still hides from afar. This world measures
that, with the same flight plus a hover over each plate, the case of an
aircraft stopping over a pad to land on it.

  plates    four solid platforms of 14 x 14 m in a north-south row at
            east 40 m, centres north -36, -12, +12, +36, each pierced by
            four square pits down to the ground: 2 m wide and 1 m deep,
            2 m and 2 m, 3 m and 1 m, 3 m and 2 m (pit depth = plate
            height). Pit cells start at 3 and 9 (2 m) or 2 and 9 (3 m)
            of 14, two or three cells from the edges
  distance  every pit is 33 m or more from the take-off point, and the
            transits run 21 m from the plates, beyond the lidar's 18 m

Kept from tools/make_world_pits.py: the plate construction (plate_parts,
solid boxes, cell-aligned pits), the flat ground, the row layout.

Outputs
    worlds/pits2.sdf        PX4 default world with the plates injected
    worlds/pits2_truth.npz  per cell truth: the keys of pits_truth.npz,
                            plus 'pit' (pit index per cell, -1 off the
                            pits) and 'plate_widths'

Run from the workspace root:
    python3 tools/make_world_pits2.py
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_world import PX4_WORLDS  # noqa: E402
from make_world_pits import plate_parts  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT_SDF = ROOT / 'worlds' / 'pits2.sdf'
OUT_TRUTH = ROOT / 'worlds' / 'pits2_truth.npz'

WORLD_NAME = 'pits2'
HALF = 80.0
RESOLUTION = 1.0
PLATE_SIDE = 14
PLATE_X = 40.0             # Gazebo ENU: x east, y north
PLATES = [
    # centre y (north), pit width, pit start cells, depth
    (-36.0, 2, (3, 9), 1.0),
    (-12.0, 2, (3, 9), 2.0),
    (12.0, 3, (2, 9), 1.0),
    (36.0, 3, (2, 9), 2.0),
]
ZONE_NAMES = ['flat', 'top', 'pit']


def pit_mask(side, width, starts):
    """Pit cells of a square plate and the index of the pit of each."""
    mask = np.zeros((side, side), dtype=bool)
    ids = np.full((side, side), -1, dtype=np.int8)
    k = 0
    for a in starts:
        for b in starts:
            mask[a:a + width, b:b + width] = True
            ids[a:a + width, b:b + width] = k
            k += 1
    return mask, ids


def build_parts():
    """Every box of the second calibration world."""
    parts = []
    for p, (cy, width, starts, depth) in enumerate(PLATES):
        pits, _ = pit_mask(PLATE_SIDE, width, starts)
        parts += plate_parts(f'plate{p}', PLATE_X - PLATE_SIDE / 2.0,
                             cy - PLATE_SIDE / 2.0, PLATE_SIDE, depth, pits)
    return parts


def build_sdf(parts):
    """PX4's default world with the plates injected before </world>."""
    src = (PX4_WORLDS / 'default.sdf').read_text()
    tag = '<world name="default">'
    if tag not in src:
        raise SystemExit(
            f'world tag not found in {PX4_WORLDS / "default.sdf"}')
    src = src.replace(tag, f'<world name="{WORLD_NAME}">', 1)
    block = ('    <model name="pits2_terrain">\n'
             '      <static>true</static>\n'
             '      <link name="terrain">\n'
             + ''.join(parts) +
             '      </link>\n'
             '    </model>\n')
    cut = src.rindex('</world>')
    return src[:cut] + block + src[cut:]


def build_truth():
    """Per cell truth, axis 0 east, axis 1 north, Gazebo ENU, exact."""
    n = int(round(2 * HALF / RESOLUTION))
    centres = -HALF + (np.arange(n) + 0.5) * RESOLUTION
    height = np.zeros((n, n))
    zone = np.zeros((n, n), dtype=np.int8)
    plate = np.full((n, n), -1, dtype=np.int8)
    pit = np.full((n, n), -1, dtype=np.int8)
    for p, (cy, width, starts, depth) in enumerate(PLATES):
        mask, ids = pit_mask(PLATE_SIDE, width, starts)
        i0 = int(round((PLATE_X - PLATE_SIDE / 2.0 + HALF) / RESOLUTION))
        j0 = int(round((cy - PLATE_SIDE / 2.0 + HALF) / RESOLUTION))
        sl = np.s_[i0:i0 + PLATE_SIDE, j0:j0 + PLATE_SIDE]
        height[sl] = np.where(mask, 0.0, depth)
        zone[sl] = np.where(mask, ZONE_NAMES.index('pit'),
                            ZONE_NAMES.index('top'))
        plate[sl] = p
        pit[sl] = np.where(mask, 4 * p + ids, -1)
    return centres, height, zone, plate, pit


def main():
    parts = build_parts()
    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    OUT_SDF.write_text(build_sdf(parts))
    centres, height, zone, plate, pit = build_truth()
    np.savez(OUT_TRUTH, centres=centres, truth=height, truth_mean=height,
             truth_max=height, zone=zone, resolution=RESOLUTION,
             zone_names=np.array(ZONE_NAMES), plate=plate, pit=pit,
             plate_depths=np.array([p[3] for p in PLATES]),
             plate_widths=np.array([p[1] for p in PLATES]))
    print(f'{OUT_SDF}  ({len(parts)} boxes)')
    print(f'{OUT_TRUTH}  ({zone.shape[0]} x {zone.shape[1]} cells)')
    for p, (cy, width, starts, depth) in enumerate(PLATES):
        on = plate == p
        print(f'plate {p}: centre E {PLATE_X:+.1f} N {cy:+.1f}, pits '
              f'{width} m wide, {depth:.1f} m deep, '
              f'{len(np.unique(pit[on & (pit >= 0)]))} pits, '
              f'{int((on & (zone == 2)).sum())} pit cells')


if __name__ == '__main__':
    main()
