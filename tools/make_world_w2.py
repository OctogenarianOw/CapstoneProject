#!/usr/bin/env python3
"""Generate W2, the world of rare landing zones, and its exact truth.

Spec 5.x: W2 is "large, mostly not landable, two or three safe zones far
apart"; it puts C1 under tension, the safe zones being far and the margin
used up fast. W1 validates the terrain classifier on varied hazards; W2
has another job, to make the energy problem the one that decides the
outcome. Hence the choices below.

  ground   a corrugated field: strips 4 m wide along y, sloping alternately
           up and down along x at 20 deg, a triangle wave of period 8 m
           and 1.46 m amplitude. 20 deg is more than three times the 6 deg
           threshold, so the classifier is not near a boundary anywhere
           (S8 calls a false accept severe beyond twice the threshold): a
           mission on W2 measures C1 and C2, not the classifier. At the
           ridges and valleys a 3 x 3 window still sees a step of 0.36 m,
           above the 0.24 m threshold, so no line of the field passes
  zones    three flat pads of 16 x 16 m at ground level: the take-off
           pad at the origin and two pads 62 and 59 m from it in opposite
           directions, 117 m apart, placed off the diagonal so that no
           frontier choice is a tie by symmetry. 16 m is two periods,
           the smallest width that keeps the pad edges on zero crossings
           of the field (no step on the slope side). Measured on the
           truth with the detection chain (footprint 1.5 m): each pad
           keeps 14 x 12 landable cells and holds a whole footprint, so
           the min-zone rule can confirm it; no slope cell passes the
           cell test (smallest analysed slope 8.6 deg, step 0.36 m)
  extent   the field covers 176 x 176 m, the exploration area 120 x 120 m
           (area_half_m 60): beyond the area the lidar sees 18 m, and the
           field goes 28 m further, so the flat ground plane outside never
           enters the map as a landing zone

Rejected: a heightmap, whose interpolation the truth would have to
reproduce, against boxes whose truth is exact by construction (rule 2 of
tools/make_world.py); random rough tiles over the whole field, 30976
boxes against 52, too heavy for the software-rendered lidar of WSL2.

Outputs
    worlds/w2.sdf        PX4 default world with the terrain injected
    worlds/w2_truth.npz  per cell ground truth, same keys as W1

Run from the workspace root:
    python3 tools/make_world_w2.py
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_world import box_pair, PX4_WORLDS  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT_SDF = ROOT / 'worlds' / 'w2.sdf'
OUT_TRUTH = ROOT / 'worlds' / 'w2_truth.npz'

WORLD_NAME = 'w2'
HALF = 88.0                # terrain half side, a multiple of the period
RESOLUTION = 1.0
SUBSAMPLES = 10
PERIOD = 8.0               # one strip up, one strip down
STRIP = PERIOD / 2.0
SLOPE_DEG = 20.0
THICK = 4.0                # > rise / cos(slope), so no cavity below a strip
PAD = 16.0                 # side of a landing pad, two periods
PADS = [
    # name, centre x (multiple of the period), centre y; Gazebo ENU
    ('home', 0.0, 0.0),
    ('pad_ne', 48.0, 40.0),
    ('pad_sw', -40.0, -48.0),
]
ZONE_NAMES = ['flat', 'slope']
COLOUR = '0.55 0.47 0.37 1'


def in_pad(x, y):
    """Mask of the points inside a landing pad."""
    inside = np.zeros(np.shape(x), dtype=bool)
    for _name, cx, cy in PADS:
        inside |= ((np.abs(x - cx) <= PAD / 2.0)
                   & (np.abs(y - cy) <= PAD / 2.0))
    return inside


def height_field(x, y):
    """Top surface height and zone id at points (x, y), Gazebo ENU."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    u = np.mod(x + HALF, PERIOD)
    tri = np.where(u < STRIP, u, PERIOD - u)
    h = np.tan(np.radians(SLOPE_DEG)) * tri
    field = (np.abs(x) <= HALF) & (np.abs(y) <= HALF) & ~in_pad(x, y)
    h = np.where(field, h, 0.0)
    zone = np.where(field, 1, 0)
    return h, zone


def y_segments(x0, x1):
    """Intervals of y covered by the field over the strip [x0, x1]."""
    cuts = sorted((cy - PAD / 2.0, cy + PAD / 2.0) for _n, cx, cy in PADS
                  if cx - PAD / 2.0 < x1 and cx + PAD / 2.0 > x0)
    out, start = [], -HALF
    for lo, hi in cuts:
        if lo > start:
            out.append((start, lo))
        start = max(start, hi)
    if start < HALF:
        out.append((start, HALF))
    return out


def build_parts():
    """One pitched box per strip segment, as SDF fragments."""
    th = np.radians(SLOPE_DEG)
    top = STRIP * np.tan(th)
    parts = []
    n = int(round(2 * HALF / STRIP))
    for k in range(n):
        x0 = -HALF + k * STRIP
        cx = x0 + STRIP / 2.0
        rising = k % 2 == 0
        # Same construction as the ramps of W1: the upper face is the
        # surface, centred at z = top / 2; the box centre sits THICK / 2
        # below it along the face normal, (-sin, 0, cos) for a rising
        # strip (pitch -th), (sin, 0, cos) for a falling one (pitch +th).
        sx = -1.0 if rising else 1.0
        pitch = -th if rising else th
        for s, (y0, y1) in enumerate(y_segments(x0, x0 + STRIP)):
            parts.append(box_pair(
                f'strip_{k:02d}_{s}',
                (cx - sx * THICK / 2.0 * np.sin(th),
                 (y0 + y1) / 2.0,
                 top / 2.0 - THICK / 2.0 * np.cos(th),
                 0.0, pitch, 0.0),
                (STRIP / np.cos(th), y1 - y0, THICK),
                COLOUR))
    return parts


def build_sdf(parts):
    """PX4's default world with the terrain injected before </world>."""
    src = (PX4_WORLDS / 'default.sdf').read_text()
    tag = '<world name="default">'
    if tag not in src:
        raise SystemExit(f'world tag not found in {PX4_WORLDS / "default.sdf"}')
    src = src.replace(tag, f'<world name="{WORLD_NAME}">', 1)
    block = ('    <model name="w2_terrain">\n'
             '      <static>true</static>\n'
             '      <link name="terrain">\n'
             + ''.join(parts) +
             '      </link>\n'
             '    </model>\n')
    cut = src.rindex('</world>')
    return src[:cut] + block + src[cut:]


def build_truth():
    """Per cell ground truth, axis 0 east, axis 1 north, Gazebo ENU."""
    n = int(round(2 * HALF / RESOLUTION))
    m = SUBSAMPLES
    s = -HALF + (np.arange(n * m) + 0.5) * (RESOLUTION / m)
    sx, sy = np.meshgrid(s, s, indexing='ij')
    h, _ = height_field(sx, sy)
    h = h.reshape(n, m, n, m)
    centres = -HALF + (np.arange(n) + 0.5) * RESOLUTION
    cx, cy = np.meshgrid(centres, centres, indexing='ij')
    _, zone = height_field(cx, cy)
    return centres, h.mean(axis=(1, 3)), h.max(axis=(1, 3)), zone


def main():
    parts = build_parts()
    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    OUT_SDF.write_text(build_sdf(parts))
    centres, truth_mean, truth_max, zone = build_truth()
    np.savez(OUT_TRUTH, centres=centres, truth=truth_mean,
             truth_mean=truth_mean, truth_max=truth_max,
             zone=zone.astype(np.int8), resolution=RESOLUTION,
             zone_names=np.array(ZONE_NAMES))
    print(f'{OUT_SDF}  ({len(parts)} boxes)')
    print(f'{OUT_TRUTH}  ({zone.shape[0]} x {zone.shape[1]} cells)')
    for zid, name in enumerate(ZONE_NAMES):
        mask = zone == zid
        print(f'{name:6s} {int(mask.sum()):6d} cells, height '
              f'{truth_mean[mask].min():.3f} to {truth_max[mask].max():.3f} m')
    for name, cx, cy in PADS:
        print(f'pad {name:7s} centre E {cx:+6.1f} N {cy:+6.1f}, '
              f'{np.hypot(cx, cy):5.1f} m from take-off')


if __name__ == '__main__':
    main()
