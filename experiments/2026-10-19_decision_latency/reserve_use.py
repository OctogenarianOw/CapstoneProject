#!/usr/bin/env python3
"""
Where the reserve went at the end of a mission. Offline, from the logs.

I1 asks the energy left to cover the return and the landing with the
reserve R untouched. This script splits the end of a mission into what
I1 counts and what it does not:

  return decision -> return path published   hover I1 does not count
  commit decision -> land command published  hover I1 does not count
  land command -> end of the log             against E_land calibrated

and reports the energy left at the end against R. Energy at a time is
read from the energy monitor values logged with each cycle, linearly
interpolated between cycles.

    python3 reserve_use.py SESSION_DIR [SESSION_DIR ...]
"""

import json
from pathlib import Path
import sys

import numpy as np


def energy_at(cycles, t):
    """Energy left at time t, interpolated between logged cycles."""
    ts = np.array([c['t'] for c in cycles])
    rs = np.array([c['remaining_j'] for c in cycles], dtype=float)
    return float(np.interp(t, ts, rs))


print(f'{"session":<44} {"R kJ":>5} {"left kJ":>7} {"R used":>6} '
      f'{"ret wait s":>10} {"land wait s":>11} {"landing kJ":>10} '
      f'{"E_land kJ":>9}')
for d in sys.argv[1:]:
    m = next((Path(d) / 'missions').iterdir())
    cfg = json.loads((m / 'config.json').read_text())
    cycles = [json.loads(x) for x in (m / 'cycles.jsonl').open()
              if json.loads(x)['remaining_j'] is not None]
    events = [json.loads(x) for x in (m / 'events.jsonl').open()]
    capacity = cycles[0]['remaining_j'] + (cycles[0]['consumed_j'] or 0.0)
    reserve = cfg['reserve_fraction'] * capacity
    actions = [e for e in events if e['kind'] == 'action']
    ret = next((e['t'] for e in events if e['kind'] == 'return'), None)
    ret_sent = next((a['t'] for a in actions if ret is not None
                     and a['t'] >= ret and a['action'] in ('path', 'land')),
                    None)
    commit = next((e['t'] for e in events if e['kind'] == 'state'
                   and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
    land = next((a['t'] for a in actions if a['action'] == 'land'
                 and commit is not None and a['t'] >= commit), None)
    left = cycles[-1]['remaining_j']
    landing = energy_at(cycles, land) - left if land is not None else np.nan
    print(f'{d[-44:]:<44} {reserve / 1e3:5.1f} {left / 1e3:7.2f} '
          f'{max(0.0, reserve - left) / 1e3:6.2f} '
          f'{(ret_sent - ret) if ret_sent else np.nan:10.1f} '
          f'{(land - commit) if land else np.nan:11.1f} '
          f'{landing / 1e3:10.2f} {cfg["energy_model"]["e_land_j"] / 1e3:9.2f}')
