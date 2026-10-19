#!/usr/bin/env python3
"""
Compare the online energy monitor with the offline probe of one flight.

Probe: rotors.csv of energy_probe_v2 (the instrument of 17 Oct and of
the calibration), integrated here with rotor_energy, total and per
flight phase. Monitor: the last /energy/state line of rec.log, and the
monitor power averaged over the hover phase, aligned on the probe clock
by the end of the record.

    python3 compare.py SESSION_DIR
"""

from pathlib import Path
import sys

import numpy as np
from uav_core.rotor_energy import EnergyIntegrator, shaft_power

d = Path(sys.argv[1])
rotors = np.genfromtxt(d / 'rotors.csv', delimiter=',', names=True)
power = np.array([shaft_power([r[f'w{k}'] for k in range(4)])
                  for r in rotors])
i = EnergyIntegrator()
for t, p in zip(rotors['t_wall'], power):
    i.add(t, p)
phases = {name: float(t) for t, name in (
    line.split(',') for line in (d / 'phases.csv').read_text().split()[1:])}

lines = [ln for ln in (d / 'rec.log').read_text().splitlines()
         if ln.count(',') >= 9]
fields = lines[-1].split(',')
# header stamp sec, nanosec, frame_id, then the message fields in order
capacity, consumed, remaining, power_w = map(float, fields[3:7])
samples, gaps = int(fields[7]), int(fields[8])

print(f'probe:   {i.consumed_j:9.1f} J over {i.samples} samples, '
      f'{i.gaps} gaps')
print(f'monitor: {consumed:9.1f} J over {samples} samples, {gaps} gaps')
print(f'difference {consumed - i.consumed_j:+.1f} J, '
      f'{100 * (consumed - i.consumed_j) / i.consumed_j:+.3f} %')
a, b = phases['HOVER_PRE'], phases['TRANSIT']
sel = (rotors['t_wall'] > a) & (rotors['t_wall'] < b)
print(f'hover power over {b - a:.1f} s, probe: {power[sel].mean():.1f} W '
      f'(calibrated P_hover 285.6 W)')
print(f'monitor remaining {remaining:.0f} J of {capacity:.0f} J')
