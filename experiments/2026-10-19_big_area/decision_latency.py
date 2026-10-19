#!/usr/bin/env python3
"""
Hover that C1 did not budget, from the logs of one session. Offline.

C1 charges each frontier visit with its planned flight plus dwell_s of
hover at arrival, for settling and for the next decision. This script
measures what the aircraft actually spent between two paths, from the
trajectory executor's own lines in ros.log (wall clock, one line per
path received, with the planned duration of the profile it flies):

  hover after path k = receipt of path k+1 - (receipt of path k
                       + planned duration of path k)

The profile is time parameterised, so receipt plus planned duration is
the arrival to within the settling time (0.06 s in calibration). The
excess over dwell_s is hover C1 never counted; times P_hover it is
energy. Also counts the paths received more than once: a resend of the
path being flown restarts it from rest where the aircraft is, one more
leg the plan did not contain.

    python3 decision_latency.py SESSION_DIR [DWELL_S]
"""

from pathlib import Path
import re
import sys

import numpy as np

P_HOVER_W = 285.636     # energy_model.CALIBRATION_2026_10_19
LINE = re.compile(r'\[trajectory_executor[^\]]*\] \[INFO\] \[([0-9.]+)\] '
                  r'\[trajectory_executor\]: path (\d+): (\d+) legs, '
                  r'([0-9.]+) m, ([0-9.]+) s')

session = Path(sys.argv[1])
dwell = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
log = session / 'ros.log'
if not log.exists():
    log = session / 'exe.log'
received = []
for line in log.open(errors='replace'):
    m = LINE.search(line)
    if m:
        received.append((float(m[1]), int(m[2]), int(m[3]), float(m[4]),
                         float(m[5])))
if not received:
    sys.exit(f'no executor path line in {log}')

first = {}
resends = []
for t, pid, legs, length, dur in received:
    if pid in first:
        resends.append((pid, t - first[pid][0], length, first[pid][3]))
    else:
        first[pid] = (t, legs, length, length, dur)

ids = sorted(first)
t0 = first[ids[0]][0]
rows = []
for a, b in zip(ids[:-1], ids[1:]):
    ta, _, _, _, dur = first[a]
    hover = first[b][0] - (ta + dur)
    rows.append((ta - t0, a, dur, hover))
hover = np.array([r[3] for r in rows])
excess = np.clip(hover - dwell, 0.0, None)

print(f'{session.name}: {len(ids)} paths, {len(resends)} resends, '
      f'dwell_s {dwell:.1f}')
print(f'{"t s":>7} {"path":>4} {"planned s":>9} {"hover s":>8} '
      f'{"unbudgeted s":>12}')
for (t, pid, dur, h), x in zip(rows, excess):
    print(f'{t:7.1f} {pid:4d} {dur:9.1f} {h:8.1f} {x:12.1f}')
print(f'hover between paths: median {np.median(hover):.1f} s, max '
      f'{hover.max():.1f} s')
print(f'unbudgeted hover: {excess.sum():.0f} s in {int((excess > 0).sum())}'
      f' of {len(excess)} intervals, {P_HOVER_W * excess.sum() / 1000:.1f} '
      f'kJ at P_hover')
mid = [r for r in resends if r[1] > 0.5]
print(f'resends: {len(resends)}, of which {len(mid)} more than 0.5 s after '
      f'the first receipt (restart in flight)')
for pid, dt, length, length0 in mid:
    print(f'   path {pid}: resent {dt:.1f} s later, {length0:.1f} m -> '
          f'{length:.1f} m left')
