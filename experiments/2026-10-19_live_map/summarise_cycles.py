#!/usr/bin/env python3
"""
Summarise a mission log: unknown ground, frontiers and timing over time.

Reads cycles.jsonl of one mission directory and prints one line every
STEP seconds, then the cycle time statistics.

    python3 summarise_cycles.py MISSION_DIR [STEP_S]
"""

from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

run = Path(sys.argv[1])
step = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
cycles = [json.loads(line) for line in (run / 'cycles.jsonl').open()]
print(f'{len(cycles)} cycles over {cycles[-1]["t"] - cycles[0]["t"]:.0f} s')
print(f'{"t s":>6} {"alt m":>6} {"unknown":>8} {"seen":>6} {"kept":>5} '
      f'{"rejected":<30} {"landing":>8} {"best gain":>9}')
next_t = cycles[0]['t']
for c in cycles:
    if c['t'] < next_t and c is not cycles[-1]:
        continue
    next_t = c['t'] + step
    kept = [f for f in c['frontiers'] if not f['rejected']]
    reasons = Counter(f['rejected'] for f in c['frontiers'] if f['rejected'])
    best = max((f['gain'] for f in kept), default=0)
    print(f'{c["t"]:6.1f} {-c["vehicle_ned"][2]:6.1f} '
          f'{c["unknown_in_area"]:8d} {c["seen_in_area"]:6d} '
          f'{len(kept):5d} {str(dict(reasons)):<30} '
          f'{c["landing_cells"]:8d} {best:9d}')
total = np.array([sum(c['timings_ms'].values()) for c in cycles])
print(f'cycle time: median {np.median(total):.0f} ms, p95 '
      f'{np.percentile(total, 95):.0f} ms, max {total.max():.0f} ms')
for key in cycles[0]['timings_ms']:
    v = np.array([c['timings_ms'][key] for c in cycles])
    print(f'  {key:<10} median {np.median(v):5.1f} ms, max {v.max():6.1f} ms')
