#!/usr/bin/env python3
"""
Replanning on an unreachable target: before and after, on W3.

Before: the W3 missions of the code of dronePropre in
../2026-10-22_i4_cells_flights (w3_before_rN). After: runs/w3_after_rN,
the branch reselect. Per mission, with the measures of
../2026-10-22_i4_cells_flights/analyse.py (point rule of I4 on both
sides, the registry of dronePropre): end reason, I1 violations,
coverage, energy left over the reserve, landing cell. And the two that
this change is about: aborts on a target no path reaches (need inf),
which ended the mission before, and replans, which take their place.

    python3 analyse.py
"""

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
I4 = HERE.parent / '2026-10-22_i4_cells_flights'
sys.path.insert(0, str(I4))
import analyse as base  # noqa: E402


def events(run):
    """Events of the mission of a run directory."""
    mdir = next((run / 'missions').iterdir())
    return [json.loads(x) for x in (mdir / 'events.jsonl').read_text()
            .splitlines() if x.strip()]


def row(run, label):
    """Measures of one mission."""
    r = base.mission(run, 'w3', 'before')
    ev = events(run)
    r['label'] = label
    r['inf_aborts'] = sum(1 for e in ev if e.get('kind') == 'abort'
                          and e.get('need_j') == 'inf')
    r['replans'] = sum(1 for e in ev if e.get('kind') == 'replan')
    r['duration'] = max(e['t'] for e in ev)
    return r


def main():
    rows = [row(p, 'before') for p in sorted(I4.glob('runs/w3_before_r*'))]
    rows += [row(p, 'after') for p in sorted(HERE.glob('runs/w3_after_r*'))]
    print(f'{"mission":<22} {"end":<28} {"t end":>6} {"viol":>4} '
          f'{"cover":>6} {"left-R":>7} {"inf aborts":>10} {"replans":>7} '
          f'{"landed on":<14}')
    for r in rows:
        land = r['landing'] + (' SEVERE' if r['landing_severe'] else '')
        print(f'{r["label"] + " " + r["run"]:<22} {r["end"]:<28} '
              f'{r["duration"]:6.0f} {r["violations"]:>4} '
              f'{r["coverage"]:5.1f}% {r["left_over_reserve"]:7.0f} '
              f'{r["inf_aborts"]:>10} {r["replans"]:>7} {land:<14}')
    for label in ('before', 'after'):
        sel = [r for r in rows if r['label'] == label]
        if sel:
            print(f'{label}: n={len(sel)}, missions ended on an unreachable '
                  f'target {sum(r["inf_aborts"] > 0 for r in sel)}, replans '
                  f'{sum(r["replans"] for r in sel)}, violations '
                  f'{sum(r["violations"] for r in sel)}, landings on a severe '
                  f'cell {sum(r["landing_severe"] for r in sel)}, coverage '
                  f'median {np.median([r["coverage"] for r in sel]):.1f} %')


if __name__ == '__main__':
    main()
