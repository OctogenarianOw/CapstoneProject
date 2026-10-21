#!/usr/bin/env python3
"""
P5: is the map smeared along the flight by a pose taken after the scan?

Rebuilds the elevation map offline from a recorded flight (rosbag of
/lidar/points, /fmu/out/vehicle_local_position_v1 and
/fmu/out/vehicle_attitude) with the frame chain, filters, variances and
grid of cloud_world, but with the pose taken at a chosen time: the
reception time of the cloud minus a lag D, interpolated on the pose
stream indexed by its own reception time. D = 0 is what cloud_world
does (the latest pose received), so the first check is that the replay
at D = 0 reproduces the map cloud_world wrote during the flight.

Measure: the edges of the plates of worlds/pits.sdf. Two maps per D,
one from the clouds taken while flying north along the lines, one from
those flying south. Spill = map height in the first ground row past a
plate edge over the plate height, averaged over the columns inside the
plate and over the four plates. The direction-dependent part is
    A(D) = ((north - south spill) going north
            - (north - south spill) going south) / 2
A pose taken after the scan moves every point forward along the flight,
so A(0) > 0 if the hypothesis holds, and A(D*) = 0 at the lag D* that
puts the pose back at the scan. Also measured, independently of the
map: for each cloud, the sample time of the latest pose received
minus the stamp of the cloud, if PX4 time and simulation time agree
(checked on the data, not assumed).

Criterion, fixed before the flight: the hypothesis stands if the replay
at D = 0 reproduces cloud_world, A(0) is positive in every flight, a D*
in [0, 0.4] s brings A to zero, and D* agrees with the stamp lag within
one lidar period (0.1 s). It is refuted if no D in the sweep brings A
to zero.

    python3 remap.py BAG_DIR RUN_DIR      # RUN_DIR holds elevation_map.npz
"""

import csv
from pathlib import Path
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

from uav_core.cloud_world import quat_to_matrix
from uav_core.elevation_grid import ElevationGrid

# cloud_world defaults (src/uav_core/uav_core/cloud_world.py)
MIN_RANGE, MAX_RANGE = 0.3, 20.0
SENSOR_OFFSET = np.array([0.0, 0.0, 0.15])
SIGMA_RANGE, SIGMA_ANGULAR = 0.02, 0.009
MIN_FUSE_HEIGHT = 2.0
GRID_SIZE = 200.0
# Region kept for the replay: the plates (north -42 to 42, east 34 to
# 46) and the ground reference, with room for the pose shifts tried.
REGION = (-48.0, 48.0, 28.0, 60.0)
PLATES = [(-36.0, 0.5), (-12.0, 1.0), (12.0, 1.5), (36.0, 2.0)]
HALF = 6.0                 # plate half side
COLS = (36, 44)            # east cells inside the plates, away from E/W
GROUND = (-4, 4, 48, 56)   # north, east cells of bare ground, reference
LAGS = np.round(np.arange(0.0, 0.401, 0.025), 3)
CRUISE = 3.0               # m/s, a cloud counts as north or south above


def read_bag(path):
    """Clouds, positions, attitudes, clock, with reception times (s)."""
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
            clouds.append((t, m.header.stamp.sec + m.header.stamp.nanosec
                           * 1e-9, decode(m)))
        elif topic == '/fmu/out/vehicle_local_position_v1':
            m = deserialize_message(data, VehicleLocalPosition)
            pos.append((t, m.timestamp_sample * 1e-6, m.x, m.y, m.z,
                        m.vx, m.vy))
        elif topic == '/fmu/out/vehicle_attitude':
            m = deserialize_message(data, VehicleAttitude)
            att.append((t, m.timestamp_sample * 1e-6, *m.q))
    return clouds, np.array(pos), np.array(att), np.array(clock)


def decode(msg):
    """Points in body FRD plus the sensor offset, range-filtered."""
    offsets = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    pts = np.stack([buf[:, offsets[k]:offsets[k] + 4].copy()
                    .view(np.float32).ravel() for k in 'xyz'], axis=1)
    norms = np.linalg.norm(pts, axis=1)
    keep = (norms > MIN_RANGE) & (norms < MAX_RANGE)
    return (pts[keep] * np.array([1.0, -1.0, -1.0]) + SENSOR_OFFSET,
            norms[keep])


def pose_at(pos, att, t):
    """Position interpolated and attitude nearest at reception time t."""
    p = np.array([np.interp(t, pos[:, 0], pos[:, k]) for k in (2, 3, 4)])
    q = att[min(np.searchsorted(att[:, 0], t), len(att) - 1), 2:6]
    return p, q / np.linalg.norm(q)


def build(clouds, pos, att, lag, select=None):
    """Grid from the clouds (all, or those select keeps) at a pose lag."""
    grid = ElevationGrid(size_m=GRID_SIZE, resolution=1.0)
    n0, n1, e0, e1 = REGION
    for k, (t, _, (pts, norms)) in enumerate(clouds):
        if select is not None and not select[k]:
            continue
        p, q = pose_at(pos, att, t - lag)
        if -p[2] < MIN_FUSE_HEIGHT:
            continue
        ned = pts @ quat_to_matrix(q).T + p
        inside = ((ned[:, 0] > n0) & (ned[:, 0] < n1)
                  & (ned[:, 1] > e0) & (ned[:, 1] < e1))
        sigma = SIGMA_RANGE + SIGMA_ANGULAR * norms[inside]
        grid.update(ned[inside], (sigma ** 2).astype(np.float32))
    return grid


def spills(grid):
    """North and south spill, mean over the plates, fraction of height."""
    up = -grid.elevation.astype(float)
    ox, oy = grid.origin_x, grid.origin_y
    g = up[int(GROUND[0] - ox):int(GROUND[1] - ox),
           int(GROUND[2] - oy):int(GROUND[3] - oy)]
    ref = np.nanmedian(g)
    cols = slice(int(COLS[0] - oy), int(COLS[1] - oy))
    north, south = [], []
    for c, d in PLATES:
        north.append(np.nanmean(up[int(c + HALF - ox), cols] - ref) / d)
        south.append(np.nanmean(up[int(c - HALF - 1 - ox), cols] - ref) / d)
    return float(np.mean(north)), float(np.mean(south))


def main():
    bag, run = Path(sys.argv[1]), Path(sys.argv[2])
    clouds, pos, att, clock = read_bag(bag)
    print(f'{len(clouds)} clouds, {len(pos)} positions, {len(att)} '
          f'attitudes, {len(clock)} clock messages')
    print(f'pose reception - PX4 stamp: median '
          f'{np.median(pos[:, 0] - pos[:, 1]):.4f} s (PX4 on the system '
          f'clock if small and positive)')
    if len(clock):
        # Offset system - simulation clock: the lower envelope of the
        # reception minus the stamp of /clock, whose transport is short.
        t0 = np.percentile(clock[:, 0] - clock[:, 1], 5)
        age = np.array([t - stamp for t, stamp, _ in clouds]) - t0
        used = []
        for t, stamp, _ in clouds:
            i = np.searchsorted(pos[:, 0], t) - 1
            if i >= 0:
                used.append(pos[i, 1] - (stamp + t0))
        print(f'cloud age at reception (system clock): median '
              f'{np.median(age):.3f} s, IQR [{np.percentile(age, 25):.3f}, '
              f'{np.percentile(age, 75):.3f}]')
        print(f'latest pose sample - scan time (system clock): median '
              f'{np.median(used):.3f} s, IQR '
              f'[{np.percentile(used, 25):.3f}, '
              f'{np.percentile(used, 75):.3f}]: the stamp lag')

    # Without /clock (r1), only the spread of reception minus stamp can
    # be read: the offset between the two clocks is unknown.
    recv_cloud = np.array([t - stamp for t, stamp, _ in clouds])
    q1, q3 = np.percentile(recv_cloud, [25, 75])
    print(f'cloud reception - cloud stamp (system - sim clock), IQR '
          f'width {q3 - q1:.3f} s')

    # Replay check at lag 0 against the map cloud_world wrote.
    full = build(clouds, pos, att, 0.0)
    live = np.load(run / 'elevation_map.npz')
    n0, n1, e0, e1 = REGION
    ox, oy = full.origin_x, full.origin_y
    win = np.s_[int(n0 + 6 - ox):int(n1 - 6 - ox),
                int(e0 + 6 - oy):int(e1 - 6 - oy)]
    a, b = full.elevation[win], live['elevation'][win]
    ok = np.isfinite(a) & np.isfinite(b) & (live['count'][win] > 20)
    print(f'replay at lag 0 against cloud_world: {int(ok.sum())} cells, '
          f'median |diff| {np.median(np.abs(a[ok] - b[ok])):.4f} m, '
          f'p95 {np.percentile(np.abs(a[ok] - b[ok]), 95):.4f} m')

    # Direction of flight at each cloud.
    vn = np.array([np.interp(t, pos[:, 0], pos[:, 5]) for t, _, _ in clouds])
    north, south = vn > CRUISE, vn < -CRUISE
    print(f'clouds flying north {int(north.sum())}, south '
          f'{int(south.sum())}')
    rows = []
    print(f'{"lag s":>6} {"N: north":>9} {"N: south":>9} {"S: north":>9} '
          f'{"S: south":>9} {"A":>7}')
    for lag in LAGS:
        nn, ns = spills(build(clouds, pos, att, lag, north))
        sn, ss = spills(build(clouds, pos, att, lag, south))
        a_lag = ((nn - ns) - (sn - ss)) / 2.0
        rows.append({'lag_s': lag, 'north_map_north': nn,
                     'north_map_south': ns, 'south_map_north': sn,
                     'south_map_south': ss, 'A': a_lag})
        print(f'{lag:6.3f} {nn:9.3f} {ns:9.3f} {sn:9.3f} {ss:9.3f} '
              f'{a_lag:+7.3f}')
    with open(run / 'lag_sweep.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    a = np.array([r['A'] for r in rows])
    cross = np.flatnonzero((a[:-1] > 0) & (a[1:] <= 0))
    if cross.size:
        k = cross[0]
        d_star = LAGS[k] + a[k] / (a[k] - a[k + 1]) * (LAGS[k + 1] - LAGS[k])
        print(f'D* = {d_star:.3f} s (A crosses zero)')
    else:
        print('A does not cross zero in the sweep')


if __name__ == '__main__':
    main()
