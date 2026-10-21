#!/usr/bin/env python3
"""
P5: the lag read on the stamps, with a local clock offset.

For each lidar scan of a bag with /clock (r2 on): the offset between the
system clock and the simulation clock is the lower envelope of the
reception time minus the stamp of /clock within 2 s of the scan, not
over the whole flight. remap.py used one offset for the flight, and on
r3 that offset jumped by about 0.85 s once (the simulation paused), so
its stamp lag was meaningless. Prints, per bag, the real-time factor of
the simulation, the age of a scan at its reception, and the sample time
of the latest PX4 pose received minus the scan stamp: the part of the
map lag D* (remap.py) that the stamps account for.

The header stamp of a PointCloud2 is read from the first bytes of its
CDR form (encapsulation, then sec and nanosec), which spares decoding
the points.

    python3 stamp_lag.py BAG_DIR [BAG_DIR ...]
"""

import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from px4_msgs.msg import VehicleLocalPosition
from rosgraph_msgs.msg import Clock

WINDOW_S = 2.0


def read(path):
    """Clock, scan stamps and pose sample times, with reception times."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)),
                rosbag2_py.ConverterOptions('', ''))
    clock, scans, poses = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        if topic == '/clock':
            m = deserialize_message(data, Clock)
            clock.append((t, m.clock.sec + m.clock.nanosec * 1e-9))
        elif topic == '/lidar/points':
            sec = int.from_bytes(data[4:8], 'little', signed=True)
            nsec = int.from_bytes(data[8:12], 'little')
            scans.append((t, sec + nsec * 1e-9))
        elif topic == '/fmu/out/vehicle_local_position_v1':
            m = deserialize_message(data, VehicleLocalPosition)
            poses.append((t, m.timestamp_sample * 1e-6))
    return np.array(clock), np.array(scans), np.array(poses)


def main():
    for path in sys.argv[1:]:
        clock, scans, poses = read(path)
        rtf = ((clock[-1, 1] - clock[0, 1]) / (clock[-1, 0] - clock[0, 0]))
        ages, lags = [], []
        for t, stamp in scans:
            near = np.abs(clock[:, 0] - t) < WINDOW_S
            offset = np.min(clock[near, 0] - clock[near, 1])
            i = np.searchsorted(poses[:, 0], t) - 1
            if i < 0:
                continue
            ages.append(t - (stamp + offset))
            lags.append(poses[i, 1] - (stamp + offset))
        ages, lags = np.array(ages), np.array(lags)
        print(f'{path}: real-time factor {rtf:.3f}; scan age at reception '
              f'median {np.median(ages):.3f} s IQR '
              f'[{np.percentile(ages, 25):.3f}, '
              f'{np.percentile(ages, 75):.3f}]; latest pose sample - scan '
              f'stamp median {np.median(lags):.3f} s IQR '
              f'[{np.percentile(lags, 25):.3f}, '
              f'{np.percentile(lags, 75):.3f}]')


if __name__ == '__main__':
    main()
