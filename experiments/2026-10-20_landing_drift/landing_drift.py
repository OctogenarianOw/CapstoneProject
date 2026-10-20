#!/usr/bin/env python3
"""
How far the aircraft lands from where LAND was commanded. Offline.

For every committed landing (COMMIT_LANDING, not EMERGENCY) of the
mission directories given: the position at the land action, linearly
interpolated from the vehicle positions logged each cycle, the speed
over the cycle around it, the final position, and the length of the
return path the manager planned before committing (0 when the aircraft
was over a landable cell already). The landing precision of P4 (p95
0.18 m, 2026-10-19_landing_p4) was measured on an aircraft at rest;
this reports what happens when it is not.

    python3 landing_drift.py RUN_DIR [RUN_DIR ...]

where each RUN_DIR holds missions/<date>/.
"""

import json
from pathlib import Path
import sys

import numpy as np

rows = []
for arg in sys.argv[1:]:
    run = Path(arg)
    mission = next((run / 'missions').iterdir())
    events = [json.loads(x) for x in (mission / 'events.jsonl').open()]
    cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
    land = next((e for e in events
                 if e['kind'] == 'action' and e['action'] == 'land'), None)
    commit = next((e for e in events if e['kind'] == 'state'
                   and e['new'] == 'COMMIT_LANDING'), None)
    if land is None or commit is None:
        continue
    returns = [e for e in events if e['kind'] == 'return']
    ts = np.array([c['t'] for c in cycles])
    pos = np.array([c['vehicle_ned'][:2] for c in cycles])
    at = np.array([np.interp(land['t'], ts, pos[:, k]) for k in (0, 1)])
    k = int(np.clip(np.searchsorted(ts, land['t']), 1, len(ts) - 1))
    speed = np.linalg.norm(pos[k] - pos[k - 1]) / max(1e-3, ts[k] - ts[k - 1])
    rows.append((f'{run.parent.parent.name}/{run.name}',
                 float(np.linalg.norm(pos[-1] - at)), float(speed),
                 returns[-1]['length_m'] if returns else float('nan')))

drift = np.array([r[1] for r in rows])
print(f'{len(rows)} committed landings')
print(f'drift from the LAND position: median {np.median(drift):.2f} m, '
      f'p95 {np.percentile(drift, 95):.2f} m, max {drift.max():.2f} m, '
      f'over 2 m: {int((drift > 2).sum())}, over 0.5 m: '
      f'{int((drift > 0.5).sum())}')
print(f'{"mission":<44} {"drift m":>7} {"speed m/s":>9} {"return m":>8}')
for name, d, v, length in sorted(rows, key=lambda r: -r[1]):
    if d > 0.5:
        print(f'{name:<44} {d:7.2f} {v:9.2f} {length:8.1f}')
