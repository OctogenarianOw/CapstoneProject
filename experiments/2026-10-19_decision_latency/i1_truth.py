#!/usr/bin/env python3
"""
Does I1 tell the truth at the return decision? Offline, from the logs.

I1 predicts, at every cycle, the energy left above the reserve after
the return and the landing. At the cycle the manager turns to RETURNING
that prediction can be checked against what happened: the energy left
at the end of the log minus the reserve. Both definitions of I1 are
computed on every flight, since cycles.jsonl logs overhead_s with the
margin (mission_policy, 4f7e635):

  before 4f7e635  B - R - (E_path + E_land)
  from 4f7e635    B - R - (E_path + P_hover overhead_s + E_land)

and the version the flight ran is the logged one. Also given with E_land
at the p95 of mission landings (energy_model.PLANNING_2026_10_20).

    python3 i1_truth.py SESSION_DIR [SESSION_DIR ...]
"""

import json
from pathlib import Path
import sys

P_HOVER = 285.636
E_LAND_CAL, E_LAND_P95 = 3199.5, 3387.0

print(f'{"session":<14} {"git":<8} {"t ret s":>7} {"ovh s":>6} '
      f'{"I1 old":>7} {"I1 new":>7} {"new, p95":>8} {"actual":>7}  kJ')
for arg in sys.argv[1:]:
    run = Path(arg)
    mission = next((run / 'missions').iterdir())
    cfg = json.load((mission / 'config.json').open())
    e_land = cfg['energy_model']['e_land_j']
    reserve = cfg['reserve_fraction'] * 20000.0
    cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
    cycles = [c for c in cycles if c['remaining_j'] is not None]
    ret = next(c for c in cycles if c['state'] == 'RETURNING')
    if 'overhead_s' not in ret:
        print(f'{run.name:<14} overhead_s not logged, skipped')
        continue
    git = next(x.split()[1][:7] for x in (run / 'setup.txt').open()
               if x.startswith('git '))
    charged = git not in ('e029ebd', '87a7035', 'a4ac769')
    hover = P_HOVER * ret['overhead_s']
    logged = ret['i1_margin_j']
    old = logged + hover if charged else logged
    new = logged if charged else logged - hover
    p95 = new - (E_LAND_P95 - e_land)
    actual = cycles[-1]['remaining_j'] - reserve
    print(f'{run.name:<14} {git:<8} {ret["t"]:>7.1f} '
          f'{ret["overhead_s"]:>6.1f} {old / 1e3:>7.2f} {new / 1e3:>7.2f} '
          f'{p95 / 1e3:>8.2f} {actual / 1e3:>7.2f}')
