#!/usr/bin/env python3
"""
Summary of one autonomous mission from its log directory (spec S17, 5.3).

Reads config.json, events.jsonl and cycles.jsonl and prints the state
timeline, every path sent, coverage over time, energy, the I1 margin
and violations, the registry, the end reason and the landing point.
Nothing is recomputed from the simulation: the log is enough (S17).

    python3 analyse_mission.py MISSION_DIR [STEP_S]
"""

import json
import math
from pathlib import Path
import sys

import numpy as np

run = Path(sys.argv[1])
step = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
config = json.loads((run / 'config.json').read_text())
events = [json.loads(line) for line in (run / 'events.jsonl').open()]
cycles = [json.loads(line) for line in (run / 'cycles.jsonl').open()]


def num(v):
    """Read a logged float, 'inf' strings included."""
    return float(v) if v is not None else math.nan


print(f'mission {run.name}: mode {config["mode"]}, policy '
      f'{config["policy"]}, alpha {config["alpha"]}, reserve '
      f'{config["reserve_fraction"]}, git {config.get("git_head", "")[:7]}')
print('\n== states')
for e in events:
    if e['kind'] == 'state':
        print(f'{e["t"]:7.1f} s  {e["old"]:<15} -> {e["new"]:<15} {e["why"]}')

print('\n== paths sent')
for e in events:
    if e['kind'] == 'action' and e.get('waypoints'):
        wp = e['waypoints']
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(wp[:-1], wp[1:]))
        print(f'{e["t"]:7.1f} s  path {e["path_id"]:3d}: {len(wp) - 1} '
              f'legs, {length:5.1f} m, to N {wp[-1][0]:+6.1f} E '
              f'{wp[-1][1]:+6.1f}')
for kind in ('blacklist', 'abort', 'return', 'emergency_landing',
             'invariant', 'return_timeout'):
    sel = [e for e in events if e['kind'] == kind]
    if sel:
        print(f'\n== {kind}: {len(sel)}')
        for e in sel[:8]:
            rest = {k: v for k, v in e.items() if k not in ('t', 'kind')}
            print(f'{e["t"]:7.1f} s  {rest}')

print('\n== coverage and energy over time')
print(f'{"t s":>7} {"state":<15} {"unknown":>7} {"seen":>6} {"frontiers":>9}'
      f' {"left kJ":>8} {"I1 kJ":>7} {"confirmed":>9}')
next_t = cycles[0]['t']
for c in cycles:
    if c['t'] < next_t and c is not cycles[-1]:
        continue
    next_t = c['t'] + step
    kept = sum(1 for f in c['frontiers'] if not f['rejected'])
    conf = sum(1 for z in c['zones'] if z['state'] == 'CONFIRMED')
    print(f'{c["t"]:7.1f} {c["state"]:<15} {c["unknown_in_area"]:7d} '
          f'{c["seen_in_area"]:6d} {kept:9d} '
          f'{num(c["remaining_j"]) / 1000:8.1f} '
          f'{num(c["i1_margin_j"]) / 1000:7.1f} {conf:9d}')

margins = np.array([num(c['i1_margin_j']) for c in cycles
                    if c['i1_margin_j'] is not None])
last = cycles[-1]
# Every cell of the exploration area is either seen or unknown, so their
# sum is the area in cells whatever area_half_m the mission flew.
area = last['seen_in_area'] + last['unknown_in_area']
print('\n== result')
print(f'duration {cycles[-1]["t"] - cycles[0]["t"]:.0f} s, coverage '
      f'{100 * last["seen_in_area"] / area:.1f} % of the area, unknown '
      f'left {last["unknown_in_area"]} cells')
print(f'energy consumed {num(last["consumed_j"]) / 1000:.1f} kJ, left '
      f'{num(last["remaining_j"]) / 1000:.1f} kJ')
if margins.size:
    print(f'I1 margin min {np.nanmin(margins) / 1000:.2f} kJ, violations '
          f'{last["violations"]}')
ends = [e for e in events if e['kind'] == 'state'
        and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')]
if ends:
    print(f'end: {ends[-1]["new"]}, {ends[-1]["why"]}')
v = last['vehicle_ned']
print(f'final position N {v[0]:+.1f} E {v[1]:+.1f} D {v[2]:+.1f}')
zones = {z['id']: z for z in last['zones']}
for z in zones.values():
    if z['state'] == 'CONFIRMED':
        d = math.hypot(v[0] - z['touchdown_ne'][0],
                       v[1] - z['touchdown_ne'][1])
        kind = 'home' if z['provisional'] else '%.0f m2' % z['area_m2']
        print(f'  confirmed zone {z["id"]}: touchdown '
              f'{tuple(round(x, 1) for x in z["touchdown_ne"])}, {kind},'
              f' {d:.1f} m from the final position')
total = np.array([sum(c['timings_ms'].values()) for c in cycles])
print(f'cycle time median {np.median(total):.0f} ms, max {total.max():.0f} ms'
      f' over {len(cycles)} cycles')
