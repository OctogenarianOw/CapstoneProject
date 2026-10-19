#!/usr/bin/env python3
"""
Touchdown moves and pass demotions of the registry, replayed offline.

Feeds a mission log through uav_core.slz_registry as the manager did:
the vehicle position of every cycle of cycles.jsonl and the latest map
snapshot (every 10 s) through the manager's own landing detection. The
registry is whichever uav_core is first on PYTHONPATH, so the same
script measures two versions on the same input:

    python3 replay_flicker.py MISSION_DIR                 # installed
    PYTHONPATH=~/uav_dev/src/uav_core:$PYTHONPATH \
        python3 replay_flicker.py MISSION_DIR             # worktree

Reports, over the non-provisional zones: touchdown moves between two
cycles, those over 5 m and the largest; CONFIRMED -> CANDIDATE
demotions whose reason is the pass count, and all demotions; cycles
during which at least one non-provisional zone was CONFIRMED.
"""

import json
import math
from pathlib import Path
import sys

import numpy as np
import uav_core.slz_registry as registry
from uav_core.observation import classify
from uav_core.terrain_analysis import analyse
from uav_core.world_model import detect_landing

run = Path(sys.argv[1])
config = json.loads((run / 'config.json').read_text())
cycles = [json.loads(x) for x in (run / 'cycles.jsonl').open()]
snaps = sorted((float(p.stem.split('_')[1]), p)
               for p in (run / 'maps').glob('map_*.npz'))

reg = registry.SlzRegistry(footprint_m=config['footprint_m'],
                           sensor_radius_m=config['sensor_radius_m'])
reg.add_provisional((0.0, 0.0), cycles[0]['t'])
current = None
last_td, moves = {}, []
pass_demotions = demotions = confirmed_cycles = n = 0
for c in cycles:
    if c['state'] in ('INIT', 'TAKEOFF', 'LANDED'):
        continue
    usable = [p for ts, p in snaps if ts <= c['t']]
    if not usable:
        continue
    if usable[-1] != current:
        current = usable[-1]
        m = np.load(current)
        origin = (float(m['origin_north']), float(m['origin_east']))
        res = float(m['resolution'])
        var, cnt = m['variance'], m['count']
        landing = detect_landing(analyse(m['elevation'], cnt, res),
                                 m['elevation'], var, classify(cnt, var),
                                 res, origin, config['footprint_m'])
    for e in reg.update(landing, var, cnt, res, origin,
                        c['vehicle_ned'][:2], c['t']):
        if e.old == registry.CONFIRMED and e.new == registry.CANDIDATE:
            demotions += 1
            pass_demotions += 'pass' in e.reason
    n += 1
    zones = reg.active()
    confirmed_cycles += any(z.state == registry.CONFIRMED for z in zones)
    for z in zones:
        if z.zone_id in last_td:
            d = math.dist(z.touchdown_ne, last_td[z.zone_id])
            if d > 0:
                moves.append(d)
        last_td[z.zone_id] = z.touchdown_ne

moves = np.array(moves) if moves else np.zeros(1)
print(f'{run.parent.parent.name}/{run.name} with {registry.__file__}')
print(f'  touchdown moves {int((moves > 0).sum())}, over 5 m '
      f'{int((moves > 5).sum())}, largest {moves.max():.1f} m')
print(f'  demotions CONFIRMED -> CANDIDATE {demotions}, on the pass count '
      f'{pass_demotions}')
print(f'  cycles with a discovered zone CONFIRMED {confirmed_cycles} of {n}')
final = sorted(reg.confirmed(), key=lambda z: -z.area_m2)
print('  confirmed at the end: ' + ', '.join(
    f'{z.zone_id} ({"home" if z.provisional else f"{z.area_m2:.0f} m2"},'
    f' {z.n_passes} passes)' for z in final))
