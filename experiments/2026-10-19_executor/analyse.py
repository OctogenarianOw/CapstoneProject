#!/usr/bin/env python3
"""
Tracking of the executor test, from the probe state and the phases.

Per phase window: horizontal distance flown (integral of |v_xy|), peak
horizontal speed, peak horizontal acceleration (finite difference of
the EKF velocity, smoothed over 0.2 s), altitude range.

    python3 analyse.py SESSION_DIR
"""

from pathlib import Path
import sys

import numpy as np

d = Path(sys.argv[1])
s = np.genfromtxt(d / 'state.csv', delimiter=',', names=True)
phases = [(float(t), name) for t, name in (
    line.split(',') for line in (d / 'phases.csv').read_text().split()[1:])]
t = s['t_wall']
v = np.hypot(s['vx'], s['vy'])
k = 10
vx = np.convolve(s['vx'], np.ones(k) / k, mode='same')
vy = np.convolve(s['vy'], np.ones(k) / k, mode='same')
acc = np.hypot(np.gradient(vx, t), np.gradient(vy, t))
print(f'{"phase":<10} {"dur s":>6} {"flown m":>8} {"v max":>6} '
      f'{"a max":>6} {"alt min":>7} {"alt max":>7}')
for (t0, name), (t1, _) in zip(phases[:-1], phases[1:]):
    sel = (t > t0) & (t < t1)
    if sel.sum() < 2:
        continue
    flown = np.trapz(v[sel], t[sel])
    print(f'{name:<10} {t1 - t0:6.1f} {flown:8.1f} {v[sel].max():6.2f} '
          f'{acc[sel].max():6.2f} {-s["z"][sel].max():7.2f} '
          f'{-s["z"][sel].min():7.2f}')
