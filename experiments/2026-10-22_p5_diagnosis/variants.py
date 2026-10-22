#!/usr/bin/env python3
"""
P5 fix v3 failed on a stable clock: which part of the pose fails?

2026-10-22_p5_stable_clock: with the pose at the scan time (branch
p5-interp, v3), P5 goes from W1's ramps, but on the plates the spill
changes side (A +0.44 -> -0.25) and the flat ground of W1 gets worse
(RMS 0.25 -> 0.30 m, recall 94 -> 61 %), every scan at the scan time,
no clock step. v3 interpolates the position AND the attitude at the
scan stamp minus a calibration term of 0.085 s. The term was measured
on the position (the PX4 estimate ahead of the Gazebo truth,
2026-10-21_p5_latency); applied to the attitude too, during the turns
and accelerations it would tilt the scan by degrees, tens of
centimetres at 15 m, and spoil flat ground. Hypothesis, not measured.

Offline, on the bags of 2026-10-21_p5_latency that hold /clock (r2,
r3; r1 has none), the chain of cloud_world rebuilds the map with the
pose taken five ways, the clock offsets estimated as v3 does (system
minus simulation: minimum over the last 0.3 s of /clock; system minus
PX4: median over the last 2 s of positions, reception minus stamp),
positions and attitudes keyed by timestamp_sample as v3 keys them:

  latest       last position and attitude received (dronePropre)
  v3           both at the scan time minus 0.085 s
  att_no_term  position at scan time - 0.085 s, attitude at scan time
  att_latest   position at scan time - 0.085 s, attitude last received
  no_term      both at the scan time

Each differs from the one before by one thing. Measured: A, the
direction-dependent spill of the plate edges (remap.py), a position
effect; and the RMS of the flat ground east and west of the plates
about its median, which an attitude error raises. The replay of
`latest` against the map cloud_world wrote checks the chain.

    python3 variants.py BAG_DIR RUN_DIR
"""

from collections import deque
from pathlib import Path
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, str(Path.home() / 'ros2_ws/droneAfter/experiments'
                       / '2026-10-21_p5_latency'))
import remap  # noqa: E402
from uav_core.cloud_world import quat_to_matrix  # noqa: E402
from uav_core.elevation_grid import ElevationGrid  # noqa: E402

TERM = 0.085
VARIANTS = ('latest', 'v3', 'att_no_term', 'att_latest', 'no_term')
GROUND_E = ((30.0, 32.0), (49.0, 58.0))     # east bands of bare ground
GROUND_N = (-44.0, 44.0)


def read(path):
    """Clouds, positions, attitudes, clock, with reception times."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)),
                rosbag2_py.ConverterOptions('', ''))
    clouds, pos, att, clock = [], [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        t = t * 1e-9
        if topic == '/clock':
            m = deserialize_message(data, Clock)
            clock.append((t, m.clock.sec + m.clock.nanosec * 1e-9))
        elif topic == '/lidar/points':
            m = deserialize_message(data, PointCloud2)
            clouds.append((t, m.header.stamp.sec
                           + m.header.stamp.nanosec * 1e-9, remap.decode(m)))
        elif topic == '/fmu/out/vehicle_local_position_v1':
            m = deserialize_message(data, VehicleLocalPosition)
            pos.append((t, m.timestamp_sample * 1e-6, m.timestamp * 1e-6,
                        m.x, m.y, m.z, m.vx, m.vy))
        elif topic == '/fmu/out/vehicle_attitude':
            m = deserialize_message(data, VehicleAttitude)
            att.append((t, m.timestamp_sample * 1e-6, *m.q))
    return clouds, np.array(pos), np.array(att), np.array(clock)


def nlerp(att, t):
    """Attitude at time t on the sample-time key, or None."""
    k = int(np.searchsorted(att[:, 1], t, side='right'))
    if k == 0 or k >= len(att):
        return None
    t0, t1 = att[k - 1, 1], att[k, 1]
    a = (t - t0) / (t1 - t0)
    q0, q1 = att[k - 1, 2:6], att[k, 2:6]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1 - a) * q0 + a * q1
    return q / np.linalg.norm(q)


def poses(clouds, pos, att, clock):
    """Pose of every cloud under every variant."""
    out = {v: [] for v in VARIANTS}
    sim = deque()
    ci = pi = 0
    px4 = deque()
    for rx, stamp, _ in clouds:
        while ci < len(clock) and clock[ci, 0] <= rx:
            sim.append((clock[ci, 0], clock[ci, 0] - clock[ci, 1]))
            ci += 1
        while sim and sim[0][0] < rx - 0.3:
            sim.popleft()
        while pi < len(pos) and pos[pi, 0] <= rx:
            px4.append((pos[pi, 0], pos[pi, 0] - pos[pi, 2]))
            pi += 1
        while px4 and px4[0][0] < rx - 2.0:
            px4.popleft()
        last_p = pos[max(0, pi - 1), 3:6]
        ka = max(0, int(np.searchsorted(att[:, 0], rx, side='right')) - 1)
        last_q = att[ka, 2:6] / np.linalg.norm(att[ka, 2:6])
        out['latest'].append((last_p, last_q))
        if not sim or not px4:
            for v in VARIANTS[1:]:
                out[v].append(None)
            continue
        t = stamp + min(s for _, s in sim) - float(np.median([o for _, o in px4]))

        def p_at(tt):
            if tt < pos[0, 1] or tt > pos[-1, 1]:
                return None
            return np.array([np.interp(tt, pos[:, 1], pos[:, k])
                             for k in (3, 4, 5)])
        pairs = {'v3': (p_at(t - TERM), nlerp(att, t - TERM)),
                 'att_no_term': (p_at(t - TERM), nlerp(att, t)),
                 'att_latest': (p_at(t - TERM), last_q),
                 'no_term': (p_at(t), nlerp(att, t))}
        for v, (p, q) in pairs.items():
            out[v].append(None if p is None or q is None else (p, q))
    return out


def build(clouds, plist, select=None):
    """Grid from the clouds with the given poses, as cloud_world fuses."""
    grid = ElevationGrid(size_m=remap.GRID_SIZE, resolution=1.0)
    n0, n1, e0, e1 = remap.REGION
    for k, (_, _, (pts, norms)) in enumerate(clouds):
        if (select is not None and not select[k]) or plist[k] is None:
            continue
        p, q = plist[k]
        if -p[2] < remap.MIN_FUSE_HEIGHT:
            continue
        ned = pts @ quat_to_matrix(q).T + p
        keep = ((ned[:, 0] > n0) & (ned[:, 0] < n1)
                & (ned[:, 1] > e0) & (ned[:, 1] < e1))
        sigma = remap.SIGMA_RANGE + remap.SIGMA_ANGULAR * norms[keep]
        grid.update(ned[keep], (sigma ** 2).astype(np.float32))
    return grid


def ground_rms(grid):
    """RMS of the bare ground about its median, metres."""
    up = -grid.elevation.astype(float)
    ox, oy = grid.origin_x, grid.origin_y
    vals = []
    for e0, e1 in GROUND_E:
        vals.append(up[int(GROUND_N[0] - ox):int(GROUND_N[1] - ox),
                       int(e0 - oy):int(e1 - oy)].ravel())
    v = np.concatenate(vals)
    v = v[np.isfinite(v)]
    return float(np.sqrt(np.mean((v - np.median(v)) ** 2))), len(v)


def main():
    bag, run = Path(sys.argv[1]), Path(sys.argv[2])
    clouds, pos, att, clock = read(bag)
    print(f'{bag.name}: {len(clouds)} clouds, {len(pos)} positions, '
          f'{len(att)} attitudes, {len(clock)} clock messages')
    print(f'PX4 timestamp - timestamp_sample, positions: median '
          f'{np.median(pos[:, 2] - pos[:, 1]) * 1e3:.1f} ms')
    plists = poses(clouds, pos, att, clock)
    vn = np.array([np.interp(t, pos[:, 0], pos[:, 6]) for t, _, _ in clouds])
    north, south = vn > remap.CRUISE, vn < -remap.CRUISE
    live = np.load(run / 'elevation_map.npz')
    print(f'{"variant":<12} {"covered":>8} {"A":>7} {"ground RMS m":>12} '
          f'{"cells":>6}')
    for v in VARIANTS:
        pl = plists[v]
        covered = sum(p is not None for p in pl) / len(pl)
        nn, ns = remap.spills(build(clouds, pl, north))
        sn, ss = remap.spills(build(clouds, pl, south))
        a = ((nn - ns) - (sn - ss)) / 2.0
        full = build(clouds, pl)
        rms, n = ground_rms(full)
        print(f'{v:<12} {covered:8.1%} {a:+7.3f} {rms:12.3f} {n:6d}')
        if v == 'latest':
            ox, oy = full.origin_x, full.origin_y
            n0, n1, e0, e1 = remap.REGION
            win = np.s_[int(n0 + 6 - ox):int(n1 - 6 - ox),
                        int(e0 + 6 - oy):int(e1 - 6 - oy)]
            x, y = full.elevation[win], live['elevation'][win]
            ok = np.isfinite(x) & np.isfinite(y) & (live['count'][win] > 20)
            print(f'  replay of latest against cloud_world: median |diff| '
                  f'{np.median(np.abs(x[ok] - y[ok])) * 1e3:.1f} mm over '
                  f'{int(ok.sum())} cells')
        rms_live, _ = ground_rms(type('G', (), {
            'elevation': live['elevation'], 'origin_x': full.origin_x,
            'origin_y': full.origin_y})) if v == 'latest' else (None, None)
        if rms_live is not None:
            print(f'  ground RMS of the live map: {rms_live:.3f} m')


if __name__ == '__main__':
    main()
