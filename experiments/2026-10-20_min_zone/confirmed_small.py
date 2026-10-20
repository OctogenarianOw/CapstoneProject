#!/usr/bin/env python3
"""
Zones the registry confirmed in flight below 9 m2, from the 1 Hz log.

At 1 m resolution and the 1.5 m footprint, a zone holding one whole
footprint element has at least 9 landable cells, so a CONFIRMED zone of
less than 9 m2 is one the rule of this experiment refuses. Lists, per
mission, each such zone with the time it was first seen confirmed, its
area and its touchdown point.

    python3 confirmed_small.py RUN_DIR [RUN_DIR ...]

where each RUN_DIR holds missions/<date>/cycles.jsonl.
"""

import json
from pathlib import Path
import sys

hits = 0
for arg in sys.argv[1:]:
    run = Path(arg)
    mission = next((run / 'missions').iterdir())
    small = {}
    for line in (mission / 'cycles.jsonl').open():
        c = json.loads(line)
        for z in c['zones']:
            if (not z['provisional'] and z['state'] == 'CONFIRMED'
                    and z['area_m2'] < 9):
                small.setdefault(z['id'], (c['t'], z['area_m2'],
                                           z['touchdown_ne']))
    if small:
        hits += 1
        for zid, (t, area, (n, e)) in small.items():
            print(f'{run.name:<12} zone {zid:<3} confirmed at {t:6.1f} s, '
                  f'{area:3.0f} m2, touchdown N {n:+6.1f} E {e:+6.1f}')
print(f'{hits} of {len(sys.argv) - 1} missions confirmed a zone under 9 m2')
