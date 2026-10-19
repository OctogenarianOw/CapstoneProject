#!/usr/bin/env python3
"""
Replay a mission log through the landing zone registry (S9), offline.

For each 1 Hz cycle of cycles.jsonl: the vehicle position of that cycle,
and the most recent map snapshot (maps/, every 10 s) run through the
same detection as the manager. Passes are counted at 1 Hz, the map
changes every 10 s. Prints the event timeline and the final registry.

    python3 replay.py MISSION_DIR [--ungated]

By default the detection is that of the manager, world_model.
detect_landing, restricted to OBSERVED cells; --ungated runs the plain
detection used before that gate, for comparison.
"""

from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
from uav_core.observation import classify
from uav_core.slz_detection import detect
from uav_core.slz_registry import SlzRegistry
from uav_core.terrain_analysis import analyse
from uav_core.world_model import detect_landing, THRESHOLDS

GATED = '--ungated' not in sys.argv
run = Path(sys.argv[1])
cycles = [json.loads(line) for line in (run / 'cycles.jsonl').open()]
snaps = sorted((float(p.stem.split('_')[1]), p)
               for p in (run / 'maps').glob('map_*.npz'))

reg = SlzRegistry()
reg.add_provisional((0.0, 0.0), cycles[0]['t'])
current, landing, layers = None, None, None
timeline = []
for c in cycles:
    t = c['t']
    usable = [p for ts, p in snaps if ts <= t]
    if not usable:
        continue
    if usable[-1] != current:
        current = usable[-1]
        m = np.load(current)
        origin = (float(m['origin_north']), float(m['origin_east']))
        res = float(m['resolution'])
        var, cnt = m['variance'], m['count']
        if GATED:
            landing = detect_landing(analyse(m['elevation'], cnt, res),
                                     m['elevation'], var,
                                     classify(cnt, var), res, origin, 1.5)
        else:
            landing = detect(analyse(m['elevation'], cnt, res),
                             m['elevation'], res, origin, footprint_m=1.5,
                             **THRESHOLDS)
    events = reg.update(landing, var, cnt, res, origin,
                        c['vehicle_ned'][:2], t)
    for e in events:
        timeline.append((t, e))
    states = Counter(z.state for z in reg.active())
    if events:
        print(f'{t:6.1f} s  alt {-c["vehicle_ned"][2]:4.1f} m  '
              + ', '.join(f'{e.zone_id}:{e.old or "-"}->{e.new}'
                          for e in events)
              + f'   active {dict(states)}')

print(f'\nzones created {reg.next_id - 1} (1 provisional), '
      f'events {len(timeline)}')
for z in sorted(reg.zones.values(), key=lambda z: -z.area_m2):
    print(f'  zone {z.zone_id:3d} {z.state:<9} area {z.area_m2:6.0f} m2 '
          f'r_in {z.inscribed_radius_m:4.1f} m passes {z.n_passes} '
          f'touchdown {tuple(round(v, 1) for v in z.touchdown_ne)} '
          f'{z.reason}')
