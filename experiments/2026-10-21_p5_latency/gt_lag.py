#!/usr/bin/env python3
"""
P5: time shift between the PX4 position estimate and the true pose.

The map lag D* (remap.py, 0.110 to 0.119 s) exceeds by about 0.08 s the
lag read on the stamps (stamp_lag.py, 0.028 to 0.032 s). Either the
lidar shows a scene older than its stamp, or the PX4 estimate is ahead
of or behind the aircraft. This compares, on a ground-truth flight
(run_gt.sh), the PX4 local position (sample time, system clock) with
the true pose Gazebo publishes (simulation clock, put on the system
clock by the local /clock offset of stamp_lag.py), Gazebo ENU turned to
NED (north = y, east = x, down = -z).

Read: the shift tau minimising the horizontal RMS of
    p_px4(t) - p_true(t - tau) - mean offset
over the samples where the aircraft flies faster than 3 m/s (a constant
offset between the EKF origin and the world origin is removed, not
interpreted). tau > 0: the estimate is late on the truth; tau < 0: it
is ahead. Also the along-track error at tau = 0, flying north and south,
which must change sign with the direction if the shift is in time.

Then the share of D* left to the lidar: the pose used is 0.03 s after
the scan stamp; if the scene the lidar shows is older than its stamp by
L and the estimate is late on the truth by tau, the pose that matches
the scene was sampled D* = 0.03 + L + tau before the one used, so
L = D* - 0.03 - tau.

    python3 gt_lag.py BAG_DIR [BAG_DIR ...]
    python3 gt_lag.py BAG_DIR --truth runs/gN/truth.jsonl

The truth of g1 is the bridged dynamic poses, without names or stamps,
timed by their reception (an upper bound on their time); from g2 on,
Gazebo's own echo with the simulation stamps (gt_echo.sh).
"""

import json
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

ODOM = '/model/x500_slz_0/odometry_with_covariance'
POSES = '/world/pits/dynamic_pose/info'
WINDOW_S = 2.0
MOVING = 3.0
SHIFTS = np.round(np.arange(-0.25, 0.2501, 0.002), 3)


def read(path):
    """Clock, PX4 positions and true positions, with reception times."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)),
                rosbag2_py.ConverterOptions('', ''))
    clock, px4, odom, tf = [], [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        if topic == '/clock':
            m = deserialize_message(data, Clock)
            clock.append((t, m.clock.sec + m.clock.nanosec * 1e-9))
        elif topic == '/fmu/out/vehicle_local_position_v1':
            m = deserialize_message(data, VehicleLocalPosition)
            px4.append((t, m.timestamp_sample * 1e-6, m.x, m.y, m.z,
                        m.vx, m.vy))
        elif topic == ODOM:
            m = deserialize_message(data, Odometry)
            p = m.pose.pose.position
            odom.append((t, m.header.stamp.sec
                         + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
        elif topic == POSES:
            # The bridge leaves the names empty and the stamps at zero
            # (g1): the first transform is the model (checked: on the
            # ground at the start, at 8 m over the transits), and the
            # reception time stands for its time, the transport of this
            # bridge being a few ms (/clock, same bridge).
            m = deserialize_message(data, TFMessage)
            if m.transforms:
                p = m.transforms[0].transform.translation
                tf.append((t, np.nan, p.x, p.y, p.z))
    return (np.array(clock), np.array(px4), np.array(odom), np.array(tf))


def to_system(stamps, recv, clock):
    """Simulation stamps to the system clock, offset taken locally."""
    out = np.empty(len(stamps))
    order = np.argsort(clock[:, 0])
    ct, cs = clock[order, 0], clock[order, 1]
    for k, (s, t) in enumerate(zip(stamps, recv)):
        a, b = np.searchsorted(ct, [t - WINDOW_S, t + WINDOW_S])
        out[k] = s + np.min(ct[a:b] - cs[a:b])
    return out


def read_echo(path):
    """True model poses from gz topic -e --json-output, stamps kept."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                m = json.loads(line)
            except ValueError:
                continue
            stamp = m.get('header', {}).get('stamp', {})
            s = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0)) * 1e-9
            for p in m.get('pose', []):
                if p.get('name') == 'x500_slz_0':
                    q = p.get('position', {})
                    out.append((np.nan, s, float(q.get('x', 0.0)),
                                float(q.get('y', 0.0)),
                                float(q.get('z', 0.0))))
    return np.array(out)


def stamps_to_system(stamps, clock):
    """Simulation stamps to the system clock, offset local in sim time."""
    order = np.argsort(clock[:, 1])
    cs, ct = clock[order, 1], clock[order, 0]
    out = np.empty(len(stamps))
    for k, s in enumerate(stamps):
        a, b = np.searchsorted(cs, [s - WINDOW_S, s + WINDOW_S])
        out[k] = s + np.min(ct[a:b] - cs[a:b])
    return out


def main():
    args = sys.argv[1:]
    echo = None
    if '--truth' in args:
        k = args.index('--truth')
        echo = read_echo(args[k + 1])
        args = args[:k] + args[k + 2:]
    for path in args:
        clock, px4, odom, tf = read(path)
        truth = odom if len(odom) else tf
        source = 'odometry' if len(odom) else 'dynamic poses'
        if echo is not None and len(echo):
            truth, source = echo, 'gz echo, simulation stamps'
        print(f'{path}: {len(px4)} PX4 positions, {len(truth)} true poses '
              f'({source}), {len(clock)} clock messages')
        if np.isnan(truth[:, 1]).all():
            t_true = truth[:, 0].copy()
            print('  true poses without stamps: reception time used')
        elif np.isnan(truth[:, 0]).all():
            t_true = stamps_to_system(truth[:, 1], clock)
        else:
            t_true = to_system(truth[:, 1], truth[:, 0], clock)
        order = np.argsort(t_true)
        t_true = t_true[order]
        ned = np.stack([truth[order, 3], truth[order, 2], -truth[order, 4]],
                       axis=1)
        t = px4[:, 1]
        speed = np.hypot(px4[:, 5], px4[:, 6])
        keep = (speed > MOVING) & (t > t_true[0] + 0.3) \
            & (t < t_true[-1] - 0.3)
        p = px4[keep, 2:5]
        rms = []
        for tau in SHIFTS:
            q = np.stack([np.interp(t[keep] - tau, t_true, ned[:, k])
                          for k in range(3)], axis=1)
            d = p[:, :2] - q[:, :2]
            d -= d.mean(axis=0)
            rms.append(np.sqrt((d ** 2).sum(axis=1).mean()))
        rms = np.array(rms)
        best = SHIFTS[np.argmin(rms)]
        q0 = np.stack([np.interp(t[keep], t_true, ned[:, k])
                       for k in range(3)], axis=1)
        offset = (p - q0).mean(axis=0)
        v = px4[keep, 5:7]
        along = ((p[:, :2] - q0[:, :2] - offset[:2]) * v).sum(axis=1) \
            / np.hypot(v[:, 0], v[:, 1])
        north, south = v[:, 0] > MOVING, v[:, 0] < -MOVING
        print(f'  tau = {best:+.3f} s (horizontal RMS {rms.min():.3f} m, '
              f'{rms[SHIFTS == 0][0]:.3f} m at tau = 0), '
              f'{int(keep.sum())} samples above {MOVING} m/s')
        print(f'  mean offset PX4 - truth: north {offset[0]:+.3f} m, east '
              f'{offset[1]:+.3f} m, down {offset[2]:+.3f} m')
        print(f'  along-track error at tau = 0: flying north '
              f'{np.median(along[north]):+.3f} m, flying south '
              f'{np.median(along[south]):+.3f} m (positive: estimate '
              f'ahead)')
        speed_m = np.median(np.hypot(v[:, 0], v[:, 1]))
        print(f'  time lead of the estimate, median along-track error over '
              f'median speed {speed_m:.2f} m/s: '
              f'{np.median(along) / speed_m:+.3f} s')


if __name__ == '__main__':
    main()
