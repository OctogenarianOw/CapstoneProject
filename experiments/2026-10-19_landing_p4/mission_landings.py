#!/usr/bin/env python3
"""
LAND-to-disarm energy of the landings flown by the manager. Offline.

For every mission directory given: the land action of events.jsonl and
the LANDED state (the executor's disarm), with the energy left at both
times interpolated from the energy monitor values logged each cycle.
The energy left does not change after the disarm (checked on missions
logged up to 716 s past it), so the interpolation at LANDED is the
energy at the disarm. Only landings committed over a registry zone
(COMMIT_LANDING) count: an emergency landing starts wherever the
budget ran out, once on the 10 deg ramp, 2.25 kJ in 10.9 s.

    python3 mission_landings.py MISSION_ROOT [MISSION_ROOT ...]

where each MISSION_ROOT holds missions/<date>/.
"""

import json
from pathlib import Path
import sys

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL

energy, time = [], []
for arg in sys.argv[1:]:
    m = next((Path(arg) / 'missions').iterdir())
    events = [json.loads(x) for x in (m / 'events.jsonl').open()]
    if not any(e['kind'] == 'state' and e['new'] == 'COMMIT_LANDING'
               for e in events):
        continue
    cycles = [json.loads(x) for x in (m / 'cycles.jsonl').open()]
    cycles = [c for c in cycles if c['remaining_j'] is not None]
    ts = np.array([c['t'] for c in cycles])
    rs = np.array([c['remaining_j'] for c in cycles])
    land = next(e['t'] for e in events
                if e['kind'] == 'action' and e['action'] == 'land')
    landed = next(e['t'] for e in events
                  if e['kind'] == 'state' and e['new'] == 'LANDED')
    energy.append(float(np.interp(land, ts, rs) - np.interp(landed, ts, rs)))
    time.append(landed - land)
e = np.array(energy)
print(f'{len(e)} committed landings: energy mean {e.mean():.0f} J, sd '
      f'{e.std(ddof=1):.0f}, min {e.min():.0f}, p95 '
      f'{np.percentile(e, 95):.0f}, max {e.max():.0f}; E_land '
      f'{MODEL.e_land_j:.0f} J, mean {100 * (e.mean() / MODEL.e_land_j - 1):+.1f}'
      f' %, p95 {100 * (np.percentile(e, 95) / MODEL.e_land_j - 1):+.1f} %')
print(f'time from the land action to LANDED, as the manager saw it: mean '
      f'{np.mean(time):.1f} s, max {np.max(time):.1f} s')
