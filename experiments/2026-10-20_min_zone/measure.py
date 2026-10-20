#!/usr/bin/env python3
"""
A minimum zone size before confirmation, against the severe false accepts.

2026-10-20_footprint_p4 found, at the 1.5 m footprint in use, one severe
false accept in 14 of the 80 last maps of 2026-10-19_campaign_w1, every
one inside a detected zone of 2 to 6 cells in the rough zone of W1. The
return field targets the nearest landable cell of any confirmed zone,
so such an island, once confirmed, is a landing target. Candidate
guard: the registry never confirms a zone below MIN cells, or (row fp)
a zone that does not hold one whole footprint element of landable
cells, the footprint test applied to the candidates themselves.

For each MIN, over the survey maps and the mission maps, with the
manager's detection (OBSERVED cells only, footprint 1.5 m): severe
cells left in zones of at least MIN cells, maps holding one, the
landable cells kept (recall against the unfiltered truth: a small true
zone dropped is a lost opportunity and counts as such), and the truth
zones that are themselves below MIN, the real opportunities the guard
would refuse.

    cd ~/ros2_ws/droneAfter
    python3 PATH/measure.py data_survey_v6.npz \
        --runs experiments/2026-10-19_campaign_w1/runs
"""

import argparse
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-10-20_footprint_p4'))

import measure as fp  # noqa: E402
from measure import v, detect, OBSERVED, VAR_MAX  # noqa: E402
from uav_core.slz_detection import footprint_ok  # noqa: E402

FOOTPRINT = 1.5
MINS = (1, 4, 7, 9, 12, 16, 25, 'fp')


def detections(d):
    state = fp.classify(d['sub_count'], d['sub_var'])
    truth = detect(d['truth_layers'], d['truth_ned'], d['res'],
                   d['sub_origin'], footprint_m=FOOTPRINT, **v.THRESHOLDS)
    found = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=FOOTPRINT,
                   variance=np.where(state == OBSERVED, d['sub_var'],
                                     np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    return truth, found


def sizes(labels):
    """Cell count of the zone each cell belongs to, 0 off zones."""
    counts = np.bincount(labels.ravel())
    counts[0] = 0
    return counts[labels]


def holds_footprint(found):
    """Candidate cells of zones holding one whole footprint element."""
    core = footprint_ok(found['candidate'], found['footprint_offsets'])
    good = np.unique(found['labels'][core])
    return found['candidate'] & np.isin(found['labels'], good[good > 0])


def evaluate(d, truth, found, minimum):
    if minimum == 'fp':
        kept = holds_footprint(found)
    else:
        kept = found['candidate'] & (sizes(found['labels']) >= minimum)
    real = truth['candidate']
    severe, _, _, _ = v.severity(d, kept & ~real, 2.0)
    return dict(severe=int(severe.sum()), accepted=int(kept.sum()),
                recall=100.0 * (kept & real).sum() / max(1, real.sum()))


def truth_zones(truth):
    return sorted(int((truth['labels'] == k).sum())
                  for k in range(1, int(truth['labels'].max()) + 1))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('maps', nargs='*')
    p.add_argument('--runs')
    args = p.parse_args()

    loaded = [(Path(m).name, fp.load_map(m)) for m in args.maps]
    if args.runs:
        loaded += [(name, fp.load_map(s))
                   for name, s in fp.last_snapshots(args.runs)]
    det = [(name, d) + detections(d) for name, d in loaded]

    _, _, truth, _ = det[0]
    zs = truth_zones(truth)
    print(f'truth zones at footprint {FOOTPRINT} m, cells: {zs}')
    print()

    surveys = [x for x in det if x[0].endswith('.npz')]
    missions = [x for x in det if not x[0].endswith('.npz')]
    print(f'{"min cells":>9} {"truth zones refused":>19} | '
          f'{"v6 severe":>9} {"v6 recall":>9} | {"maps w/ severe":>14} '
          f'{"severe cells":>12} {"recall median":>13} {"recall min":>10}')
    for minimum in MINS:
        refused = (sum(1 for s in zs if s < minimum) if minimum != 'fp'
                   else 'n/a')
        sv = [evaluate(d, t, f, minimum) for _, d, t, f in surveys]
        ms = [evaluate(d, t, f, minimum) for _, d, t, f in missions]
        v6 = sv[0] if sv else dict(severe=-1, recall=float('nan'))
        print(f'{str(minimum):>9} {refused:>19} | {v6["severe"]:>9} '
              f'{v6["recall"]:>8.1f}% | '
              f'{sum(r["severe"] > 0 for r in ms):>14} '
              f'{sum(r["severe"] for r in ms):>12} '
              f'{np.median([r["recall"] for r in ms]):>12.1f}% '
              f'{min(r["recall"] for r in ms):>9.1f}%')


if __name__ == '__main__':
    main()
