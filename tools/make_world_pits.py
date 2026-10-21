#!/usr/bin/env python3
"""Generate the pit calibration world and its exact ground truth.

Role: measure, before W3 is built, whether a pad pierced with pits looks
landable to the mapping chain when seen from afar and is refused once
seen from close, and at which pit depth. W3 (spec 5.1, the deceptive
world) needs a zone the registry confirms from a distance and a closer
observation invalidates (S9.5); the depth of its pits decides both, so
it is measured here, not chosen.

  ground    PX4's flat ground plane and nothing else: no slope and no
            roughness to confound the step the pits make
  plates    four solid platforms of 12 x 12 m in a north-south row at
            east 40 m, centres north -36, -12, +12, +36, heights 0.5,
            1.0, 1.5 and 2.0 m. Each is pierced by four pits of 1 x 1 m
            down to the ground, 4 m apart as on the W3 pad, at cells 4
            and 8 of 12: pit depth = plate height. The pits sit 3 and 4
            cells from the plate edges so that the 3 x 3 block read
            around a pit, even shifted by the one cell of P5, never
            holds the edge step of the plate. With 8 m plates and pits
            at cell 1, the filled truth still read the 0.5 to 2.0 m edge
            step around 3 of the 4 pits (analyse_pits.py --truth)
  geometry  a pit floor is the ground plane, a plate is boxes from below
            the ground to its top (rule 1 of make_world.py, nothing
            floats), every pit edge on a cell edge (rule 2)
  distance  every pit is 38 m or more from the take-off point, beyond
            the 18 m the lidar reaches at 8 m, so the first view of a
            pit is the one the flight plan gives, not the climb

Why pits and not bumps: a bump intercepts rays at any range, a pit hides
its floor from any ray flatter than depth / width, so at 8 m a pit 1 m
wide and 1 m deep shows its floor only within about 8 m. That is a
geometric argument, and this world exists to test it.

Rejected: pits narrower than a cell, whose truth would be an average of
pit and plate, against cell-aligned pits whose truth is exact.

Outputs
    worlds/pits.sdf        PX4 default world with the plates injected
    worlds/pits_truth.npz  per cell truth, keys of W1 and W2 plus
                           'plate' (index per cell, -1 off the plates)
                           and 'plate_depths'

Run from the workspace root:
    python3 tools/make_world_pits.py
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_world import box_pair, PX4_WORLDS  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT_SDF = ROOT / 'worlds' / 'pits.sdf'
OUT_TRUTH = ROOT / 'worlds' / 'pits_truth.npz'

WORLD_NAME = 'pits'
HALF = 80.0                # truth half side, inside the 200 m map
RESOLUTION = 1.0
BELOW = 0.1                # boxes start under the ground plane (rule 1)
PLATE_SIDE = 12
PLATE_X = 40.0             # Gazebo ENU: x east, y north
PLATE_YS = [-36.0, -12.0, 12.0, 36.0]
PLATE_DEPTHS = [0.5, 1.0, 1.5, 2.0]
PIT_PITCH = 4              # pits 4 m apart, as on the W3 pad
PIT_OFFSET = 4             # first pit cell, 3 to 4 cells from the edges
ZONE_NAMES = ['flat', 'top', 'pit']
COLOUR = '0.50 0.50 0.55 1'


def pit_cells(side, pitch=PIT_PITCH, offset=PIT_OFFSET):
    """Boolean (side, side) mask of the pit cells of a square plate."""
    pits = np.zeros((side, side), dtype=bool)
    idx = np.arange(offset, side, pitch)
    pits[np.ix_(idx, idx)] = True
    return pits


def plate_parts(name, x0, y0, side, height, pits, colour=COLOUR):
    """Boxes of a plate whose lower corner is (x0, y0), pits left open.

    One box per run of solid cells along y in each column of cells along
    x, so a plate with p pits per column costs at most p + 1 boxes per
    column (W2's 52 boxes are the budget the software lidar handles).
    """
    parts = []
    for i in range(side):
        j = 0
        while j < side:
            if pits[i, j]:
                j += 1
                continue
            k = j
            while k < side and not pits[i, k]:
                k += 1
            parts.append(box_pair(
                f'{name}_{i:02d}_{j:02d}',
                (x0 + (i + 0.5) * RESOLUTION,
                 y0 + (j + k) / 2.0 * RESOLUTION,
                 (height - BELOW) / 2.0, 0.0, 0.0, 0.0),
                (RESOLUTION, (k - j) * RESOLUTION, height + BELOW),
                colour))
            j = k
    return parts


def build_parts():
    """Every box of the calibration world."""
    parts = []
    pits = pit_cells(PLATE_SIDE)
    for p, (cy, depth) in enumerate(zip(PLATE_YS, PLATE_DEPTHS)):
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
    block = ('    <model name="pits_terrain">\n'
             '      <static>true</static>\n'
             '      <link name="terrain">\n'
             + ''.join(parts) +
             '      </link>\n'
             '    </model>\n')
    cut = src.rindex('</world>')
    return src[:cut] + block + src[cut:]


def build_truth():
    """Per cell truth, axis 0 east, axis 1 north, Gazebo ENU, exact.

    Every structure is aligned on the 1 m cells, so a cell is wholly
    ground, plate top or pit, and its mean and maximum are its height.
    """
    n = int(round(2 * HALF / RESOLUTION))
    centres = -HALF + (np.arange(n) + 0.5) * RESOLUTION
    height = np.zeros((n, n))
    zone = np.zeros((n, n), dtype=np.int8)
    plate = np.full((n, n), -1, dtype=np.int8)
    pits = pit_cells(PLATE_SIDE)
    for p, (cy, depth) in enumerate(zip(PLATE_YS, PLATE_DEPTHS)):
        i0 = int(round((PLATE_X - PLATE_SIDE / 2.0 + HALF) / RESOLUTION))
        j0 = int(round((cy - PLATE_SIDE / 2.0 + HALF) / RESOLUTION))
        sl = np.s_[i0:i0 + PLATE_SIDE, j0:j0 + PLATE_SIDE]
        height[sl] = np.where(pits, 0.0, depth)
        zone[sl] = np.where(pits, ZONE_NAMES.index('pit'),
                            ZONE_NAMES.index('top'))
        plate[sl] = p
    return centres, height, zone, plate


def main():
    parts = build_parts()
    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    OUT_SDF.write_text(build_sdf(parts))
    centres, height, zone, plate = build_truth()
    np.savez(OUT_TRUTH, centres=centres, truth=height, truth_mean=height,
             truth_max=height, zone=zone, resolution=RESOLUTION,
             zone_names=np.array(ZONE_NAMES), plate=plate,
             plate_depths=np.array(PLATE_DEPTHS))
    print(f'{OUT_SDF}  ({len(parts)} boxes)')
    print(f'{OUT_TRUTH}  ({zone.shape[0]} x {zone.shape[1]} cells)')
    for p, (cy, depth) in enumerate(zip(PLATE_YS, PLATE_DEPTHS)):
        on = plate == p
        print(f'plate {p}: centre E {PLATE_X:+.1f} N {cy:+.1f}, depth '
              f'{depth:.1f} m, {int((on & (zone == 2)).sum())} pits, '
              f'{int((on & (zone == 1)).sum())} top cells')


if __name__ == '__main__':
    main()
