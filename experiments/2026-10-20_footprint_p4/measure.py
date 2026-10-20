#!/usr/bin/env python3
"""
Footprint 1.2 m against 1.5 m: what the P4 change does to detection.

2026-10-19_landing_p4 measured the landing error (p95 0.180 m over 15
landings) and the airframe radius (0.386 m), so the footprint of S16 is
2 (0.386 + 0.180) = 1.13 m, rounded up to 1.2 m. Before the detector is
changed, this script measures the effect offline, with the footprint as
the only difference:

  element   cells of the structuring element, its radius rf, the
            minimum width of an isolated zone by the formula of S8 and
            on synthetic square platforms in refused ground, and the
            landing cells lost around one isolated bump
  surveys   recall, accepted, benign and severe false accepts against
            the truth on the survey maps (v6 is the reference)
  missions  the same on the last map of each mission of a campaign:
            independent flights, which S8 asks for before a zero of
            severe false accepts counts (at least three)

Evaluation as in tools/validate_slz.py and 2026-10-19_observed_gate:
the truth is detected with the same footprint (whether the aircraft
fits is a fact of the terrain), severe = truth past twice a threshold,
and the map detection is the manager's, on OBSERVED cells only
(world_model.detect_landing).

    cd ~/ros2_ws/droneAfter
    python3 PATH/measure.py data_survey_v*.npz \
        --runs experiments/2026-10-19_campaign_w1/runs
"""

import argparse
import io
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))

from uav_core.observation import classify, OBSERVED, VAR_MAX  # noqa: E402
from uav_core.slz_detection import detect, footprint_offsets  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402
import validate_slz as v  # noqa: E402

FOOTPRINTS = (1.5, 1.2)
ANALYSIS_RADIUS = 1      # terrain_analysis.analyse default, 3x3 window


# ------------------------------------------------------------- element


def element(footprint, res=1.0):
    """Cells of the element, its radius rf, minimum width of S8."""
    offsets = footprint_offsets(footprint, res)
    rf = max(abs(di) for di, _ in offsets)
    return len(offsets), rf, (2 * (ANALYSIS_RADIUS + rf) + 1) * res


def platform_min_width(footprint, res=1.0, n=31):
    """
    Smallest square platform of flat ground that keeps a landing cell.

    The platform sits in a checkerboard of 1 m steps, which the cell
    test refuses everywhere, so only the platform can hold a candidate.
    """
    i, j = np.indices((n, n))
    count = np.ones((n, n))
    for w in range(1, 12):
        elev = ((i + j) % 2).astype(float)
        lo = (n - w) // 2
        elev[lo:lo + w, lo:lo + w] = 0.5
        out = detect(analyse(elev, count, res), elev, res, (0.0, 0.0),
                     footprint_m=footprint, **v.THRESHOLDS)
        if out['candidate'].any():
            return w * res, int(out['candidate'].sum())
    return float('nan'), 0


def bump_loss(footprint, res=1.0, n=21):
    """Landing cells refused around one raised cell in flat ground."""
    elev = np.zeros((n, n))
    elev[n // 2, n // 2] = 1.0
    out = detect(analyse(elev, np.ones((n, n)), res), elev, res,
                 (0.0, 0.0), footprint_m=footprint, **v.THRESHOLDS)
    inner = ~out['candidate'][5:-5, 5:-5]
    return int(inner.sum())


# ----------------------------------------------------------------- maps


def load_map(path):
    """validate_slz data for a saved map or a mission snapshot."""
    s = np.load(path)
    ox = s['origin_x'] if 'origin_x' in s.files else s['origin_north']
    oy = s['origin_y'] if 'origin_y' in s.files else s['origin_east']
    buf = io.BytesIO()
    np.savez(buf, elevation=s['elevation'], variance=s['variance'],
             count=s['count'], resolution=s['resolution'],
             origin_x=ox, origin_y=oy)
    buf.seek(0)
    d = v.analyse_both(v.load(SimpleNamespace(
        map_file=buf, truth_file=v.DEFAULT_TRUTH, truth_layer='truth')))
    res, n = d['res'], d['n_t']
    i0 = int(round((-d['half_t'] + res / 2.0 - float(ox)) / res))
    j0 = int(round((-d['half_t'] + res / 2.0 - float(oy)) / res))
    d['sub_var'] = s['variance'][i0:i0 + n, j0:j0 + n]
    return d


def evaluate(d, footprint):
    state = classify(d['sub_count'], d['sub_var'])
    truth = detect(d['truth_layers'], d['truth_ned'], d['res'],
                   d['sub_origin'], footprint_m=footprint, **v.THRESHOLDS)
    found = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=footprint,
                   variance=np.where(state == OBSERVED, d['sub_var'],
                                     np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    mapped, real = found['candidate'], truth['candidate']
    severe, benign, _, _ = v.severity(d, mapped & ~real, 2.0)
    return dict(real=int(real.sum()),
                recall=100.0 * (mapped & real).sum() / max(1, real.sum()),
                accepted=int(mapped.sum()), benign=int(benign.sum()),
                severe=int(severe.sum()), zones=len(found['zones']))


def row(label, results):
    cells = ' '.join(f'{r["real"]:5d} {r["recall"]:6.1f}% {r["accepted"]:5d} '
                     f'{r["benign"]:4d} {r["severe"]:3d}' for r in results)
    return f'{label:<22} {cells}'


def header():
    cols = ' '.join(f'{"truth":>5} {"recall":>7} {"acc":>5} {"ben":>4} '
                    f'{"sev":>3}' for _ in FOOTPRINTS)
    fps = '   '.join(f'footprint {fp} m{"":<14}' for fp in FOOTPRINTS)
    return f'{"":<22} {fps}\n{"map":<22} {cols}'


def last_snapshots(runs):
    for run in sorted(p for p in Path(runs).iterdir() if p.is_dir()):
        snaps = sorted(run.glob('missions/*/maps/map_*.npz'))
        if snaps:
            yield run.name, snaps[-1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('maps', nargs='*')
    p.add_argument('--runs', help='campaign runs/ directory')
    args = p.parse_args()

    print('element at 1 m resolution, analysis radius 1 cell, no margin')
    print(f'{"footprint":>9} {"cells":>6} {"rf":>3} {"W min S8":>9} '
          f'{"W min platform":>15} {"cells lost to one bump":>23}')
    for fp in FOOTPRINTS:
        cells, rf, w_formula = element(fp)
        w_plat, n_cand = platform_min_width(fp)
        print(f'{fp:>9.1f} {cells:>6} {rf:>3} {w_formula:>8.1f}m '
              f'{w_plat:>6.1f}m ({n_cand} cell) {bump_loss(fp):>23}')
    print()

    if args.maps:
        print('survey maps')
        print(header())
        for path in args.maps:
            d = load_map(path)
            print(row(Path(path).name, [evaluate(d, fp) for fp in FOOTPRINTS]))
        print()

    if args.runs:
        print(f'last map of each mission in {args.runs}')
        print(header())
        per_fp = {fp: [] for fp in FOOTPRINTS}
        for name, snap in last_snapshots(args.runs):
            try:
                d = load_map(snap)
            except SystemExit as e:
                print(f'{name:<22} skipped: {e}')
                continue
            results = [evaluate(d, fp) for fp in FOOTPRINTS]
            for fp, r in zip(FOOTPRINTS, results):
                per_fp[fp].append(r)
            print(row(name, results))
        print()
        print(f'{"footprint":>9} {"maps":>5} {"with severe":>12} '
              f'{"severe total":>13} {"recall median":>14} '
              f'{"accepted median":>16} {"benign median":>14}')
        for fp, rs in per_fp.items():
            if not rs:
                continue
            print(f'{fp:>9.1f} {len(rs):>5} '
                  f'{sum(r["severe"] > 0 for r in rs):>12} '
                  f'{sum(r["severe"] for r in rs):>13} '
                  f'{np.median([r["recall"] for r in rs]):>13.1f}% '
                  f'{np.median([r["accepted"] for r in rs]):>16.0f} '
                  f'{np.median([r["benign"] for r in rs]):>14.0f}')


if __name__ == '__main__':
    main()
