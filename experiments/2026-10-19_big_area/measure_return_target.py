#!/usr/bin/env python3
"""
Return to the zone touchdown point or to the nearest landable cell?

Replays the registry over a whole mission (vehicle track at 1 Hz from
cycles.jsonl, latest map snapshot through the manager's detection), then
at chosen times compares the two ways of seeding the return field:
  touchdown  each confirmed zone seeds its interior touchdown point
  cells      each confirmed zone seeds all its landable cells (current)
Reports the I1 margin from the vehicle, and over the frontier candidates
the median E_return and how many are admissible, with the energy left
logged at that time and the mission's reserve.

    python3 measure_return_target.py MISSION_DIR T1 [T2 ...]
"""

import glob
import json
import sys
from types import SimpleNamespace

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.mission_policy import (evaluate, invariant_margin,
                                     PolicyParams, return_field)
from uav_core.slz_registry import SlzRegistry
from uav_core.world_model import build_world, WorldParams

run = sys.argv[1]
times = sorted(float(x) for x in sys.argv[2:])
config = json.load(open(f'{run}/config.json'))
cycles = [json.loads(line) for line in open(f'{run}/cycles.jsonl')]
snaps = sorted((float(p.split('_')[-1][:-4]), p)
               for p in glob.glob(f'{run}/maps/map_*.npz'))
params = WorldParams(area_half_m=config['area_half_m'])
reserve = config['reserve_fraction'] * 200000.0
reg = SlzRegistry()
reg.add_provisional((0.0, 0.0), 0.0)
current, world = None, None
pending = list(times)
for c in cycles:
    usable = [p for ts, p in snaps if ts <= c['t']]
    if not usable:
        continue
    if usable[-1] != current:
        current = usable[-1]
        m = np.load(current)
        origin = (float(m['origin_north']), float(m['origin_east']))
        res = float(m['resolution'])
    vehicle = c['vehicle_ned'][:2]
    world = build_world(m['elevation'], m['variance'], m['count'], res,
                        origin, vehicle, params)
    reg.update(world.landing, m['variance'], m['count'], res, origin,
               vehicle, c['t'])
    if not pending or c['t'] < pending[0]:
        continue
    pending.pop(0)
    b = c['remaining_j']
    zones = reg.confirmed()
    stubs = [SimpleNamespace(zone_id=z.zone_id, touchdown_ne=z.touchdown_ne)
             for z in zones]
    print(f'== t {c["t"]:.0f} s, vehicle N {vehicle[0]:+.0f} E '
          f'{vehicle[1]:+.0f}, {b / 1000:.1f} kJ left, confirmed '
          f'{[(z.zone_id, round(z.area_m2)) for z in zones]}')
    for label, zs in (('touchdown', stubs), ('cells', zones)):
        rf = return_field(world.traversable, zs, res, origin)
        margin, zid, plan = invariant_margin(world, rf, MODEL, b, reserve,
                                             res, origin)
        d = evaluate(world, rf, MODEL, b, PolicyParams(reserve_j=reserve),
                     res, origin)
        e_ret = [x.e_return for x in d.candidates if np.isfinite(x.e_return)]
        adm = sum(x.admissible for x in d.candidates)
        print(f'   {label:<10} I1 margin {margin / 1000:7.2f} kJ via zone '
              f'{zid}, return {plan.length_m if plan else float("nan"):5.0f}'
              f' m | candidates {len(d.candidates)}, median E_return '
              f'{np.median(e_ret) / 1000:5.2f} kJ, admissible {adm}')
