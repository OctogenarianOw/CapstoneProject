#!/usr/bin/env python3
"""
Time not in the plan per frontier visit, and how well a window predicts it.

For each frontier path k flown to arrival, from events.jsonl alone:

  U_k = publish of the next command - decision of path k - planned(k)

The decision time is when the manager read the energy C1 used; the next
command is the first one published after the arrival, whatever it is;
planned(k) is the profile time of path k from the energy model. U_k is
all the time C1 had to cover beyond the planned flight: the computation
before path k left, the wait for the next cycle at the frontier, and the
computation of the next decision. C1 charged dwell_s for it.

Then compares predictors of U_k known at decision k, the maximum over
the last N measured values (N = 1, 3, 5, 10), floored at dwell_s:
how often each under-predicts and by how much energy at P_hover.

    python3 overrun_series.py MISSION_DIR [DWELL_S]
"""

import json
from pathlib import Path
import sys

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL

run = Path(sys.argv[1])
dwell = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
events = [json.loads(line) for line in (run / 'events.jsonl').open()]

visits = []          # (decision t, planned s, arrival seen t, path id)
decision = None
current = None
for e in events:
    if e['kind'] == 'state' and e['new'] == 'NAVIGATING':
        decision = e['t']
    elif (e['kind'] == 'action' and e['action'] == 'path'
          and decision is not None and current is None):
        current = (decision, MODEL.path_time(e['waypoints']), e['path_id'])
        decision = None
    elif e['kind'] == 'state' and e['old'] == 'NAVIGATING':
        if e['why'] == 'arrived' and current is not None:
            visits.append(current + (e['t'],))
        current = None
    if e['kind'] == 'state' and e['new'] == 'NAVIGATING' and \
            current is not None:
        current = None

# next command published after each arrival
actions = [e for e in events if e['kind'] == 'action']
rows = []
for t_dec, planned, pid, t_arr in visits:
    nxt = next((a for a in actions if a['t'] >= t_arr
                and a.get('path_id') != pid), None)
    if nxt is None:
        continue
    rows.append((t_dec, pid, planned, nxt['t'] - t_dec - planned))

u = np.array([r[3] for r in rows])
print(f'{run.name}: {len(u)} visits flown to arrival, dwell_s {dwell}')
print(f'{"t s":>7} {"path":>4} {"planned s":>9} {"U s":>6}')
for t, pid, planned, x in rows:
    print(f'{t:7.1f} {pid:4d} {planned:9.1f} {x:6.1f}')
print(f'U median {np.median(u):.1f} s, max {u.max():.1f} s; over dwell_s '
      f'in {(u > dwell).sum()} of {len(u)}')


def window_max(k, n):
    """Return the largest of the last n values before visit k."""
    return max([dwell] + list(u[max(0, k - n):k]))


def window_rise(k, n):
    """Return the largest rise between consecutive values in the window."""
    w = u[max(0, k - n):k]
    return max([0.0] + list(np.diff(w))) if len(w) > 1 else 0.0


predictors = {'dwell_s alone': lambda k: dwell}
for n in (1, 3, 5, 10):
    predictors[f'max of last {n}'] = (lambda k, n=n: window_max(k, n))
# The one mission_fsm.MissionFSM.overhead_s implements.
predictors['max of last 5 + rise'] = (
    lambda k: max(dwell, max([0.0] + list(u[max(0, k - 5):k]))
                  + window_rise(k, 5)))
print('\npredictor              under-predicted  worst shortfall'
      '          mean charge')
for name, f in predictors.items():
    pred = np.array([f(k) for k in range(len(u))])
    short = u - pred
    worst = max(0.0, short.max())
    print(f'{name:<22} {(short > 0).sum():3d} of {len(u):<3d}      '
          f'{worst:5.1f} s, {MODEL.p_hover_w * worst / 1000:5.2f} kJ   '
          f'{pred.mean():5.1f} s')
