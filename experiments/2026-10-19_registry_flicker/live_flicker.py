#!/usr/bin/env python3
"""
Touchdown moves and pass demotions of the registry, as flown. Offline.

Reads the 1 Hz log of a mission as the manager wrote it, no replay:
cycles.jsonl gives every zone's touchdown point and pass count at every
cycle, events.jsonl its transitions. Reports, over the non-provisional
zones not RETIRED, the touchdown moves between two cycles (over 5 m,
largest), the CONFIRMED -> CANDIDATE demotions on the pass count, and
how many of them hit a zone the I1 margin relied on at the cycle before.

    python3 live_flicker.py MISSION_DIR [MISSION_DIR ...]
"""

import json
import math
from pathlib import Path
import sys

print(f'{"mission":<44} {"moves":>5} {"> 5 m":>5} {"max m":>6} '
      f'{"pass demotions":>14} {"of the zone I1 used":>19}')
for arg in sys.argv[1:]:
    run = Path(arg)
    cycles = [json.loads(x) for x in (run / 'cycles.jsonl').open()]
    events = [json.loads(x) for x in (run / 'events.jsonl').open()]
    last, moves = {}, []
    for c in cycles:
        for z in c['zones']:
            if z['provisional'] or z['state'] == 'RETIRED':
                continue
            if z['id'] in last:
                d = math.dist(z['touchdown_ne'], last[z['id']])
                if d > 0:
                    moves.append(d)
            last[z['id']] = z['touchdown_ne']
    demotions = [e for e in events if e['kind'] == 'zone'
                 and e['old'] == 'CONFIRMED' and e['new'] == 'CANDIDATE'
                 and 'pass' in e['reason']]
    # Zone the I1 margin relied on: logged in invariant events only, so
    # take the last one before each demotion.
    inv = [e for e in events if e['kind'] == 'invariant']
    relied = 0
    for d in demotions:
        before = [e for e in inv if e['t'] < d['t'] - 0.5]
        if before and before[-1]['zone_id'] == d['zone_id']:
            relied += 1
    name = f'{run.parent.parent.name}/{run.name}'[-44:]
    print(f'{name:<44} {len(moves):5d} {sum(m > 5 for m in moves):5d} '
          f'{max(moves, default=0):6.1f} {len(demotions):14d} '
          f'{relied:19d}')
