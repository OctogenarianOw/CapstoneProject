#!/usr/bin/env python3
"""
Energy against coverage over one mission, from the 1 Hz log. Offline.

For each coverage level, the time and the energy consumed when the
mission first reached it; then when the return started and why, the
landing time, the energy at the end, and the decision cycle time. Used
to choose a W2 budget where C1 binds before the world is covered.

    python3 energy_coverage.py RUN_DIR
"""

import json
from pathlib import Path
import sys

run = Path(sys.argv[1])
mission = next((run / 'missions').iterdir())
cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
events = [json.loads(x) for x in (mission / 'events.jsonl').open()]

landed = next(e['t'] for e in events
              if e['kind'] == 'state' and e['new'] == 'LANDED')
ret = next(e for e in events
           if e['kind'] == 'state' and e['new'] == 'RETURNING')
print(f'return at {ret["t"]:.1f} s ({ret["why"]}), landed at {landed:.1f} s')

area = cycles[-1]['seen_in_area'] + cycles[-1]['unknown_in_area']
marks = [0.25, 0.5, 0.6, 0.7, 0.8, 0.9, 0.95, 0.99]
k = 0
for c in cycles:
    if c['consumed_j'] is None:
        continue
    coverage = c['seen_in_area'] / area
    while k < len(marks) and coverage >= marks[k]:
        print(f'coverage {marks[k]:.2f} at {c["t"]:6.1f} s, consumed '
              f'{c["consumed_j"] / 1000:6.1f} kJ')
        k += 1
last = cycles[-1]
print(f'consumed at the end {last["consumed_j"] / 1000:.1f} kJ, left '
      f'{last["remaining_j"] / 1000:.1f} kJ')
totals = sorted(sum(c['timings_ms'].values()) for c in cycles
                if c.get('timings_ms'))
print(f'decision cycle {totals[len(totals) // 2]:.0f} ms median, '
      f'{totals[-1]:.0f} ms max')
