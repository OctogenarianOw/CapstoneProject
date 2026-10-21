#!/usr/bin/env python3
"""Generate W3, the world where a confirmed zone is invalidated in flight.

Spec 5.1 gives W3 the job of provoking the shrinking of the registry,
the critical case of S9.5, and first imagined a zone that looks safe
from afar and is not up close. Measured on 21 Oct, no such zone exists
in this sensing chain: pits 1 m wide are never seen, even from
overhead, and pits 2 to 3 m wide are seen from 10 to 17 m, as soon as
they are mapped (experiments/2026-10-21_pit_calibration and _2). There
is no window in which a pad looks safe and is not. W3 therefore puts
the change in the world rather than in the eye: a supply depot of 6
crates is placed on a confirmed zone during the mission, by a script,
at the moment the aircraft's guarantee rests on that zone and the zone
is in view (experiments/2026-10-22_w3_design). The invalidation is then
a real observation through the whole chain (lidar, map, analysis,
detection, registry), and the delay between the change and the
reaction is measured instead of assumed.

  ground  the corrugated field of W2, same construction and truth
          (make_world_w2.py), 20 deg strips, nowhere near a threshold
  pads    two pads only: the 16 m take-off pad T at the origin, and Z,
          8 m east by 12 m north, centre E 28, N 12, 30.5 m away. Near
          enough to be confirmed: I4 asks two passes, and on W2 the
          pads 62 m out were confirmed in 2 of the 50 C1 missions of
          2026-10-21_campaign_w2 (2026-10-22_w3_design). With
          one remote zone, the only fallback when Z goes is T, so the
          reaction to the loss is a return to T or a flight on to a
          target T still covers, never a switch to a third zone that
          would hide it. N 12 breaks the mirror symmetry of the field
          about the x axis. Z is small on purpose: the depot retires Z
          only if the aircraft sees all of it (a part left unseen stays
          flat in the map and keeps the zone confirmed), and the sensor
          sees about 16 m around it at 8 m. A 16 m pad (half diagonal
          11.3 m) is wholly in view only within about 5 m of its
          centre, an 8 x 12 m pad (7.2 m) within about 9 m. Its
          landable core, 6 x 8 cells, stays above the smallest
          confirmable platform, 7 m (experiments/2026-10-20_min_zone).
          East edges on zero crossings of the field, as in W2
  depot   6 crates of 2 x 2 x 1.5 m on a 4 m pitch over Z, 1 m in from
          its edges, cell aligned. Each crate refuses its own cells and
          a ring of one cell (a 1.5 m step against the 0.24 m
          threshold), and the 4 x 4 blocks tile the pad: no cell of Z
          stays landable, so the zone is no longer detected and is
          retired, not merely shrunk. A separate model,
          worlds/w3_depot.sdf, spawned at run time, not part of w3.sdf
  truths  w3_truth.npz before the depot, w3_depot_truth.npz after,
          same keys as W1 and W2; zone 2 marks the crates

Rejected: a pad that looks safe from afar (measured impossible, above);
retiring the zone in software, which skips the perception chain and
the detection delay the reserve has to cover; one large object over the
pad (a tent), whose far face is hidden from a passing aircraft at 8 m
and would leave stale flat cells that keep the zone confirmed
(geometry, not flown).

Outputs, from the workspace root:
    python3 tools/make_world_w3.py
    worlds/w3.sdf, worlds/w3_truth.npz, worlds/w3_depot.sdf,
    worlds/w3_depot_truth.npz, and a check of both truths through the
    landing detection chain
"""

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / 'src' / 'uav_core'))
from make_world import box_pair  # noqa: E402
import make_world_w2 as w2  # noqa: E402
from uav_core.slz_detection import detect, footprint_ok  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
WORLD_NAME = 'w3'
OUT_SDF = ROOT / 'worlds' / 'w3.sdf'
OUT_TRUTH = ROOT / 'worlds' / 'w3_truth.npz'
OUT_DEPOT = ROOT / 'worlds' / 'w3_depot.sdf'
OUT_DEPOT_TRUTH = ROOT / 'worlds' / 'w3_depot_truth.npz'

PADS = [
    # name, centre x, centre y, side x, side y; Gazebo ENU. x edges on
    # multiples of the 8 m period, where the field is at zero height
    ('home', 0.0, 0.0, 16.0, 16.0),
    ('pad_z', 28.0, 12.0, 8.0, 12.0),
]
Z = PADS[1]
CRATE = 2.0                # side, m
CRATE_H = 1.5              # height, m
PITCH = 4.0                # crate spacing, crate plus its refused ring
CRATE_COLOUR = '0.33 0.38 0.22 1'
ZONE_NAMES = ['flat', 'slope', 'crate']
FOOTPRINT_M = 1.5
THRESHOLDS = {'max_slope_deg': 6.0, 'max_roughness_m': 0.05,
              'max_step_m': 0.24}


def crate_centres():
    """Centres (x, y) of the crates over pad Z, Gazebo ENU."""
    cx, cy, hx, hy = w2.pad_box(Z)

    def offsets(half):
        first = -half + 1.0 + CRATE / 2.0
        return first + PITCH * np.arange(int(2.0 * half // PITCH))

    return [(cx + dx, cy + dy) for dx in offsets(hx) for dy in offsets(hy)]


def build_depot_sdf():
    """The depot as a static model whose boxes carry world poses."""
    parts = [box_pair(f'crate_{k:02d}', (x, y, CRATE_H / 2.0, 0.0, 0.0, 0.0),
                      (CRATE, CRATE, CRATE_H), CRATE_COLOUR)
             for k, (x, y) in enumerate(crate_centres())]
    return ('<?xml version="1.0" ?>\n'
            '<sdf version="1.9">\n'
            '  <model name="w3_depot">\n'
            '    <static>true</static>\n'
            '    <link name="crates">\n'
            + ''.join(parts) +
            '    </link>\n'
            '  </model>\n'
            '</sdf>\n')


def add_depot(centres, truth_mean, truth_max, zone):
    """Truth after the depot: crate cells raised, zone 2."""
    mean, top, z = truth_mean.copy(), truth_max.copy(), zone.copy()
    res = w2.RESOLUTION
    for x, y in crate_centres():
        ix = np.flatnonzero(np.abs(centres - x) < CRATE / 2.0)
        iy = np.flatnonzero(np.abs(centres - y) < CRATE / 2.0)
        assert len(ix) == len(iy) == int(CRATE / res), 'crate off the grid'
        mean[np.ix_(ix, iy)] = CRATE_H
        top[np.ix_(ix, iy)] = CRATE_H
        z[np.ix_(ix, iy)] = 2
    return mean, top, z


def landable_on_pads(centres, truth_mean):
    """Landable cells per pad, and whether a whole footprint fits."""
    res = w2.RESOLUTION
    count = np.ones(truth_mean.shape, dtype=np.int32)
    layers = analyse(truth_mean, count, res)
    land = detect(layers, truth_mean, res, (centres[0], centres[0]),
                  footprint_m=FOOTPRINT_M, **THRESHOLDS)
    core = footprint_ok(land['candidate'], land['footprint_offsets'])
    out = {}
    off = land['candidate'].copy()
    for pad in PADS:
        cx, cy, hx, hy = w2.pad_box(pad)
        box = np.ix_(np.abs(centres - cx) <= hx, np.abs(centres - cy) <= hy)
        out[pad[0]] = (int(land['candidate'][box].sum()),
                       bool(core[box].any()))
        off[np.ix_(np.abs(centres - cx) <= hx + 1.0,
                   np.abs(centres - cy) <= hy + 1.0)] = False
    out['elsewhere'] = (int(off.sum()), bool(off.any()))
    return out


def main():
    parts = w2.build_parts(PADS)
    OUT_SDF.parent.mkdir(parents=True, exist_ok=True)
    OUT_SDF.write_text(w2.build_sdf(parts, WORLD_NAME))
    centres, mean, top, zone = w2.build_truth(PADS)
    np.savez(OUT_TRUTH, centres=centres, truth=mean, truth_mean=mean,
             truth_max=top, zone=zone.astype(np.int8),
             resolution=w2.RESOLUTION, zone_names=np.array(ZONE_NAMES))
    OUT_DEPOT.write_text(build_depot_sdf())
    d_mean, d_top, d_zone = add_depot(centres, mean, top, zone)
    np.savez(OUT_DEPOT_TRUTH, centres=centres, truth=d_mean,
             truth_mean=d_mean, truth_max=d_top,
             zone=d_zone.astype(np.int8), resolution=w2.RESOLUTION,
             zone_names=np.array(ZONE_NAMES))
    print(f'{OUT_SDF}  ({len(parts)} boxes)')
    print(f'{OUT_DEPOT}  ({len(crate_centres())} crates)')
    for pad in PADS:
        cx, cy, hx, hy = w2.pad_box(pad)
        print(f'pad {pad[0]:6s} {2 * hx:.0f} x {2 * hy:.0f} m, centre '
              f'E {cx:+6.1f} N {cy:+6.1f}, {np.hypot(cx, cy):5.1f} m from '
              'take-off')
    for label, m in (('before the depot', mean), ('after the depot', d_mean)):
        print(f'truth through the detection chain, {label}:')
        for name, (n, whole) in landable_on_pads(centres, m).items():
            print(f'  {name:9s} {n:4d} landable cells, whole footprint '
                  f'{"yes" if whole else "no"}')


if __name__ == '__main__':
    main()
