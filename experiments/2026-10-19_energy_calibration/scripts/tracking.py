#!/usr/bin/env python3
"""Tracking error of a calibration flight against its planned profile.

Rebuilds the trapezoidal profile from setup.txt and compares it with the
EKF position and velocity during the transit segment. A fixed cruise
speed (S10) and a flown path matching the planned one (S12, 5 %) are
claims about tracking, so they are measured here rather than assumed
from the setpoints.

    python3 tracking.py RUN_DIR [RUN_DIR ...]
"""

import sys
import types
from pathlib import Path

import numpy as np

# calibration_flight imports ROS at module level; the profile code does
# not need it, so stub the ROS modules to reuse the exact same profile.
for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'std_msgs', 'std_msgs.msg',
             'px4_msgs', 'px4_msgs.msg'):
    stub = types.ModuleType(name)
    for attr in ('Node', 'QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy',
                 'DurabilityPolicy', 'String', 'OffboardControlMode',
                 'TrajectorySetpoint', 'VehicleCommand', 'VehicleStatus',
                 'VehicleLocalPosition', 'TimesyncStatus'):
        setattr(stub, attr, object)
    sys.modules.setdefault(name, stub)
sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibration_flight as cf  # noqa: E402


def report(run):
    setup = dict(line.partition(' ')[::2]
                 for line in (run / 'setup.txt').read_text().splitlines())
    pts = cf.parse_path(setup['path'])
    prof = cf.TrapezoidPath(pts, float(setup['speed']),
                            float(setup['accel']))
    s = np.genfromtxt(run / 'state.csv', delimiter=',', names=True)
    phases = {}
    for line in (run / 'phases.csv').read_text().splitlines()[1:]:
        t, name = line.split(',')
        phases.setdefault(name, float(t))
    a, b = phases['TRANSIT'], phases['HOVER_POST']
    sel = (s['t_wall'] > a) & (s['t_wall'] < b)
    tau = s['t_wall'][sel] - a
    pos = np.stack([s['x'][sel], s['y'][sel]], axis=1)
    vel = np.stack([s['vx'][sel], s['vy'][sel]], axis=1)
    ref = np.array([prof.at(t)[0] for t in tau])
    vref = np.array([prof.at(t)[1] for t in tau])
    err = np.hypot(*(pos - ref).T)
    speed = np.hypot(*vel.T)
    cruise = np.hypot(*vref.T) >= float(setup['speed']) - 1e-6
    flown = float(np.trapz(speed, tau))
    print(f'== {run.name}: planned {prof.length:.1f} m in '
          f'{prof.duration:.2f} s, flown {flown:.1f} m '
          f'({100 * (flown / prof.length - 1):+.1f} %) in {b - a:.2f} s')
    print(f'   position error to profile  median {np.median(err):.2f} m, '
          f'p95 {np.percentile(err, 95):.2f} m, max {err.max():.2f} m')
    if cruise.any():
        print(f'   speed while profile cruises at {setup["speed"]} m/s: '
              f'median {np.median(speed[cruise]):.2f}, '
              f'p5 {np.percentile(speed[cruise], 5):.2f}, '
              f'p95 {np.percentile(speed[cruise], 95):.2f} m/s, '
              f'{cruise.sum()} samples')
    lo = np.min(np.array(pts), axis=0)
    hi = np.max(np.array(pts), axis=0)
    overshoot = max(0.0, *(pos.max(axis=0) - hi), *(lo - pos.min(axis=0)))
    print(f'   max speed {speed.max():.2f} m/s, overshoot past the '
          f'path bounding box {overshoot:.2f} m')


if __name__ == '__main__':
    for arg in sys.argv[1:]:
        report(Path(arg))
