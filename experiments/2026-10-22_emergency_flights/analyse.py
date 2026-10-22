#!/usr/bin/env python3
"""
Emergency start search and hold: before and after, on W3.

Per mission, with the measures of ../2026-10-22_i4_cells_flights/
analyse.py (I4 per cell on both sides, the registry of dronePropre since
e9c727f): end reason, I1 violations, coverage, energy left over the
reserve, landing cell. And what this change is about: emergencies, with
their reason and the energy left over the reserve when declared, and
the drift of an emergency landing, from where LAND was sent (the event
emergency_land after the change, emergency_landing before, which sent
LAND at once) to where the aircraft touched down.

    python3 analyse.py
"""

import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-10-22_i4_cells_flights'))
import analyse as base  # noqa: E402


def row(run):
    """Measures of one mission."""
    variant = run.name.split('_')[1]
    r = base.mission(run, 'w3', 'after')
    mdir = next((run / 'missions').iterdir())
    ev = [json.loads(x) for x in (mdir / 'events.jsonl').read_text()
          .splitlines() if x.strip()]
    cyc = [json.loads(x) for x in (mdir / 'cycles.jsonl').read_text()
           .splitlines() if x.strip()]
    capacity = cyc[-1]['remaining_j'] + cyc[-1]['consumed_j']
    reserve = 0.1 * capacity
    em = [e for e in ev if e.get('kind') == 'emergency_landing']
    sent = [e for e in ev if e.get('kind') == 'emergency_land'] or em
    r.update(variant=variant, replans=sum(1 for e in ev
                                          if e.get('kind') == 'replan'),
             escapes=sum(1 for e in ev if e.get('kind') == 'escape'))
    r['emergencies'] = [e['reason'] for e in em]
    r['emergency_over_reserve'] = [
        round(min(cyc, key=lambda c: abs(c['t'] - e['t']))['remaining_j']
              - reserve) for e in em]
    drift = math.nan
    if sent:
        at = min(cyc, key=lambda c: abs(c['t'] - sent[0]['t']))
        down = [c for c in cyc if c['state'] == 'LANDED'][0]
        drift = math.dist(at['vehicle_ned'][:2], down['vehicle_ned'][:2])
    r['drift_m'] = drift
    return r


def main():
    """Table of the runs of this directory, or of the one given."""
    runs = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'runs'
    rows = [row(p) for p in sorted(runs.glob('w3_*'))]
    print(f'{"mission":<14} {"end":<20} {"viol":>4} {"cover":>6} '
          f'{"left-R":>7} {"replans":>7} {"escapes":>7} {"drift m":>7}  '
          'emergencies (J over the reserve)  landed on')
    for r in rows:
        em = ', '.join(f'{w} ({j})' for w, j in
                       zip(r['emergencies'], r['emergency_over_reserve']))
        land = r['landing'] + (' SEVERE' if r['landing_severe'] else '')
        print(f'{r["run"]:<14} {r["end"]:<20} {r["violations"]:>4} '
              f'{r["coverage"]:5.1f}% {r["left_over_reserve"]:7.0f} '
              f'{r["replans"]:>7} {r["escapes"]:>7} {r["drift_m"]:7.1f}  '
              f'{em or "-"}  {land}')
    for v in ('before', 'after'):
        sel = [r for r in rows if r['variant'] == v]
        if sel:
            early = sum(any(j > 0 for j in r['emergency_over_reserve'])
                        for r in sel)
            print(f'{v}: n={len(sel)}, emergencies with energy above the '
                  f'reserve {early}, violations '
                  f'{sum(r["violations"] for r in sel)}, landings on a severe '
                  f'cell {sum(r["landing_severe"] for r in sel)}, coverage '
                  f'median {np.median([r["coverage"] for r in sel]):.1f} %')


if __name__ == '__main__':
    main()
