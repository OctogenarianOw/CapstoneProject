#!/usr/bin/env python3
"""
How often a transit relying on pad Z comes near Z: the measure behind
the approach mode of the depot trigger (README.md, 24 Oct).

For every decision whose return relies on Z (the live trigger's own
'relies', from w3_depot.evaluate_logged on the logged decision), the
closest the aircraft comes to the centre of Z before the next decision,
from cycles.jsonl, and the I1 margin without Z at that decision (the
manager's guarantee event). A transit counts as approaching when it
comes within R_NEAR of Z, close enough for the vans placed then to be
in view before the aircraft is over Z.

    python3 approach_measure.py RUNS_DIR [RUNS_DIR ...]
"""

import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from w3_depot import (decisions, evaluate_logged, guarantee_of,  # noqa: E402
                      PADS, TARGET)

Z = (-28.0, 28.0)
R_NEAR = (12.0, 18.0)


def load(path):
    return [json.loads(x) for x in path.open() if x.strip()]


def main():
    missions = n_rely = 0
    near = {r: set() for r in R_NEAR}
    rows = []
    for d in sys.argv[1:]:
        for run in sorted(Path(d).glob('B3a*')):
            if run.name.startswith('.'):
                continue
            m = next((run / 'missions').iterdir())
            config = json.loads((m / 'config.json').read_text())
            cycles = load(m / 'cycles.jsonl')
            events = load(m / 'events.jsonl')
            missions += 1
            decs = list(decisions(cycles, events))
            for k, (cy, ch, path) in enumerate(decs):
                r = evaluate_logged(cy, ch, path, guarantee_of(events, cy, ch),
                                    config, PADS['w3'], TARGET['w3'])
                if not r.get('relies'):
                    continue
                n_rely += 1
                t_end = decs[k + 1][0]['t'] if k + 1 < len(decs) else 1e9
                seg = [c for c in cycles if cy['t'] <= c['t'] < t_end]
                dmin = min(math.dist(c['vehicle_ned'][:2], Z) for c in seg)
                rows.append((run.name, cy['t'], dmin,
                             r.get('i1_without_at_decision_j', math.nan)))
                for rr in R_NEAR:
                    if dmin <= rr:
                        near[rr].add(f'{d}/{run.name}')
    print(f'{missions} missions, {n_rely} decisions relying on Z')
    for name, t, dmin, i1 in rows:
        print(f'  {name:<11} t {t:6.1f}  closest to Z {dmin:5.1f} m  '
              f'I1 without Z {i1:7.0f} J')
    for rr in R_NEAR:
        print(f'missions with a transit relying on Z within {rr:.0f} m '
              f'of it: {len(near[rr])} of {missions}')


if __name__ == '__main__':
    main()
