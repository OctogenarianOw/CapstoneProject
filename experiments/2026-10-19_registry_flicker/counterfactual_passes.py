#!/usr/bin/env python3
"""
What the pass count would have been without I4 option b. Offline.

Since d115a97 the registry keeps the larger of the logged pass count
and the count recomputed around the current touchdown. This script
recomputes, from the 1 Hz log alone, the count around each logged
touchdown over the vehicle track up to that cycle, as the registry did
before d115a97 (SlzRegistry.count_passes, r_in 0.8 x 18 m, r_out 18 m),
and reports the cycles where it falls below the logged count, and those
where a CONFIRMED zone would then have had fewer than 2 passes: the
demotions the change prevented.

On flights flown before d115a97 the recount must equal the logged
count at every cycle; that is the check of the script itself.

    python3 counterfactual_passes.py MISSION_DIR [MISSION_DIR ...]
"""

import json
import math
from pathlib import Path
import sys

R_OUT = 18.0
R_IN = 0.8 * R_OUT
MIN_PASSES = 2


def count_passes(track, point):
    inside, passes = False, 0
    for n, e in track:
        d = math.hypot(n - point[0], e - point[1])
        if not inside and d < R_IN:
            inside, passes = True, passes + 1
        elif inside and d > R_OUT:
            inside = False
    return passes


print(f'{"mission":<44} {"zone-cycles":>11} {"recount equal":>13} '
      f'{"recount lower":>13} {"demotions prevented":>19}')
for arg in sys.argv[1:]:
    run = Path(arg)
    cycles = [json.loads(x) for x in (run / 'cycles.jsonl').open()]
    track, total, equal, lower, prevented = [], 0, 0, 0, []
    for c in cycles:
        if c.get('vehicle_ned') is None:
            continue
        track.append((c['vehicle_ned'][0], c['vehicle_ned'][1]))
        for z in c['zones']:
            if z['provisional'] or z['state'] == 'RETIRED':
                continue
            total += 1
            again = count_passes(track, z['touchdown_ne'])
            if again == z['passes']:
                equal += 1
            elif again < z['passes']:
                lower += 1
                if z['state'] == 'CONFIRMED' and again < MIN_PASSES:
                    prevented.append((round(c['t'], 1), z['id'],
                                      z['passes'], again))
    name = f'{run.parent.parent.name}/{run.name}'
    print(f'{name:<44} {total:>11} {equal:>13} {lower:>13} '
          f'{len(prevented):>19}')
    for t, zid, logged, again in prevented:
        print(f'    {t:>7.1f} s  zone {zid}: {logged} passes kept, '
              f'{again} around the current touchdown')
