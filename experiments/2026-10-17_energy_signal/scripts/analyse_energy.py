"""Rotor shaft power and PX4 battery telemetry for one recorded flight.

Shaft power per rotor in the Gazebo MulticopterMotorModel:
    thrust = k_f * w^2,  drag torque = k_m * thrust,  P = k_m * k_f * w^3
with k_f = 8.54858e-06 and k_m = 0.016 from the x500 model.sdf.
"""
import sys

import numpy as np

KF = 8.54858e-06
KM = 0.016
MASS = 2.0 + 4 * 0.016076923076923075 + 0.2      # base, rotors, lidar
G = 9.81

d = sys.argv[1]
r = np.genfromtxt(d + '/rotors.csv', delimiter=',', names=True)
s = np.genfromtxt(d + '/state.csv', delimiter=',', names=True)
b = np.genfromtxt(d + '/battery.csv', delimiter=',', names=True)

w = np.stack([r[f'w{i}'] for i in range(4)], axis=1)
t = r['t_sim']
ok = np.isfinite(t)
t, w = t[ok], w[ok]
print(f'rotor msgs {len(t)}, sim span {t[-1] - t[0]:.1f} s, '
      f'wall span {r["t_wall"][ok][-1] - r["t_wall"][ok][0]:.1f} s, '
      f'stamp field nonzero: {np.any(r["t_stamp"] != 0)}')

power = KM * KF * np.sum(w ** 3, axis=1)

# Align PX4 time on sim time using the first instant of rotor spin-up and
# the first armed state sample.
spin = np.argmax(np.mean(w, axis=1) > 50.0)
armed = s['arming'] == 2
offset = t[spin] - s['t_px4'][np.argmax(armed)]
ts = s['t_px4'] + offset

# Interpolate state on rotor timestamps.
vx = np.interp(t, ts, s['vx'])
vy = np.interp(t, ts, s['vy'])
vz = np.interp(t, ts, s['vz'])
z = np.interp(t, ts, s['z'])
arm = np.interp(t, ts, armed.astype(float)) > 0.5
vh = np.hypot(vx, vy)

w_hover_pred = np.sqrt(MASS * G / 4.0 / KF)
p_hover_pred = 4.0 * KM * KF * w_hover_pred ** 3
print(f'mass {MASS:.3f} kg, predicted hover w {w_hover_pred:.1f} rad/s, '
      f'shaft power {p_hover_pred:.1f} W')

air = arm & (z < -1.0)
hover = air & (vh < 0.2) & (np.abs(vz) < 0.1)
print(f'hover samples {hover.sum()} ({hover.sum() / max(1, air.sum()):.0%} '
      f'of airborne)')
if hover.any():
    print(f'  measured hover w  {np.mean(w[hover]):.1f} rad/s '
          f'(sd {np.std(np.mean(w[hover], axis=1)):.1f})')
    print(f'  measured hover P  {np.mean(power[hover]):.1f} W')

print('\nshaft power by horizontal speed, level flight (|vz| < 0.2 m/s)')
level = air & (np.abs(vz) < 0.2)
edges = [0, 0.2, 1, 2, 3, 4, 5, 6, 8, 12]
for lo, hi in zip(edges[:-1], edges[1:]):
    sel = level & (vh >= lo) & (vh < hi)
    if sel.sum() > 50:
        print(f'  {lo:4.1f}-{hi:4.1f} m/s  n {sel.sum():6d}  '
              f'P {np.mean(power[sel]):6.1f} W  sd {np.std(power[sel]):5.1f}')

print('\nclimb and descent')
for lo, hi, name in [(-9, -0.5, 'climb'), (0.5, 9, 'descent')]:
    sel = air & (vz >= lo) & (vz < hi)
    if sel.sum() > 20:
        print(f'  {name:8s} n {sel.sum():6d}  mean vz {np.mean(vz[sel]):+.2f}  '
              f'P {np.mean(power[sel]):6.1f} W')

# Energy over the armed airborne period.
dt = np.diff(t, prepend=t[0])
dt = np.clip(dt, 0, 0.1)
e_total = np.sum(power[air] * dt[air])
t_air = np.sum(dt[air])
dist = np.sum(vh[air] * dt[air])
p_ref = np.mean(power[hover]) if hover.any() else p_hover_pred
print(f'\nairborne {t_air:.1f} s, horizontal distance {dist:.1f} m')
print(f'  shaft energy {e_total / 1000:.2f} kJ, '
      f'hover power x time {p_ref * t_air / 1000:.2f} kJ '
      f'({p_ref * t_air / e_total:.1%} of total)')

print('\nPX4 battery_status while armed')
ta = b['t_px4'] + offset
bsel = (ta >= t[spin]) & (ta <= t[arm][-1])
if bsel.sum() > 2:
    bt = ta[bsel] - ta[bsel][0]
    rem = b['remaining'][bsel]
    slope = np.polyfit(bt, rem, 1)[0]
    print(f'  remaining {rem[0]:.4f} -> {rem[-1]:.4f} over {bt[-1]:.1f} s, '
          f'slope {slope:.3e} /s (1/3000 = {1 / 3000:.3e})')
    print(f'  current_a unique {np.unique(b["current_a"][bsel])}, '
          f'discharged_mah unique {np.unique(b["discharged_mah"][bsel])}')
