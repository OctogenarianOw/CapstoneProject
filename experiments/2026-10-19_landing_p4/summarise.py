#!/usr/bin/env python3
"""
Landing precision (S16, P4) and closed-loop landing energy (S10).

Reads sessions/*/landing.json and reports:
  - radial touchdown error to the setpoint: mean, sd, p95, max, and the
    footprint of S16, footprint = 2 (airframe radius + p95), rounded up
    to 0.1 m. Airframe radius 0.386 m, measured in the x500_base model
    of PX4: rotor hubs at (+-0.174, +-0.174) m, 0.246 m from the centre,
    blade collision box 0.279 m long, so 0.140 m of blade beyond the
    hub. The spec estimated 0.35 m
  - the energy and duration from LAND to disarm, against E_land of the
    energy model, with the p95 as the conservative value
  - drift during the descent, from the position at the LAND command

    python3 summarise.py [CAMPAIGN_DIR]
"""

import json
import math
from pathlib import Path
import sys

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
AIRFRAME_RADIUS_M = math.hypot(0.174, 0.174) + 0.2792307692307692 / 2

rows = [json.loads((d / 'landing.json').read_text())
        for d in sorted((HERE / 'sessions').iterdir())
        if (d / 'landing.json').exists()]
failed = sorted(p.name for p in (HERE / 'failed').iterdir()) \
    if (HERE / 'failed').exists() else []
print(f'{len(rows)} landings, {len(failed)} failed sessions flown again'
      f'{": " + ", ".join(failed) if failed else ""}')


def describe(name, x, unit, digits=3):
    """Print mean, sd, median, p95 and max of one quantity."""
    x = np.asarray(x, float)
    print(f'{name:<28} mean {x.mean():.{digits}f} sd {x.std(ddof=1):.{digits}f}'
          f' median {np.median(x):.{digits}f} p95 '
          f'{np.percentile(x, 95):.{digits}f} max {x.max():.{digits}f} {unit}')


err = [r['radial_error_m'] for r in rows]
describe('radial error to setpoint', err, 'm')
describe('drift during the descent', [r['drift_during_landing_m']
                                      for r in rows], 'm')
describe('error at the LAND command', [math.hypot(r['at_land_ne'][0],
                                                  r['at_land_ne'][1])
                                       for r in rows], 'm')
p95 = float(np.percentile(err, 95))
footprint = math.ceil(20.0 * (AIRFRAME_RADIUS_M + p95) - 1e-9) / 10.0
print(f'airframe radius {AIRFRAME_RADIUS_M:.3f} m, footprint '
      f'2 x ({AIRFRAME_RADIUS_M:.3f} + {p95:.3f}) = '
      f'{2 * (AIRFRAME_RADIUS_M + p95):.3f} m, rounded up {footprint:.1f} m'
      f' (in use: 1.5 m)')
energy = [r['landing_energy_j'] for r in rows]
describe('LAND to disarm energy', energy, 'J', 0)
describe('LAND to disarm time', [r['landing_time_s'] for r in rows], 's', 2)
print(f'E_land in the model {MODEL.e_land_j:.0f} J: mean measured is '
      f'{100 * (np.mean(energy) / MODEL.e_land_j - 1):+.1f} %, p95 '
      f'{100 * (np.percentile(energy, 95) / MODEL.e_land_j - 1):+.1f} %')
print('\nper landing: radial error m, drift m, energy J, time s')
for r in rows:
    print(f'  {r["radial_error_m"]:.3f} {r["drift_during_landing_m"]:.3f} '
          f'{r["landing_energy_j"]:.0f} {r["landing_time_s"]:.2f}')
