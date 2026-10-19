#!/usr/bin/env python3
"""
Frontier catalogue on map v6, whole survey and simulated early flight.

Two cases. The survey map restricted to the 60 x 60 m validation world:
the survey covered it, so the catalogue should hold little or nothing.
An early-flight map: v6 cut to the 18 m footprint seen from take-off,
everything else set to UNKNOWN, which is what the explorer faces after
its climb. Also times detect_frontiers on the 100 x 100 grid.

    python3 frontiers_on_v6.py MAP.npz
"""

import sys
import time

import numpy as np
from uav_core.frontier_detection import detect_frontiers

m = np.load(sys.argv[1])
count, var = m['count'], m['variance']
res = float(m['resolution'])
origin = (float(m['origin_x']), float(m['origin_y']))
n = count.shape[0]
ii, jj = np.indices(count.shape)
cn = origin[0] + (ii + 0.5) * res
ce = origin[1] + (jj + 0.5) * res
world = (np.abs(cn) < 30) & (np.abs(ce) < 30)


def show(label, cat, t_ms):
    print(f'== {label}: {len(cat.frontiers)} kept, {len(cat.rejected)} '
          f'rejected, unknown in area {cat.unknown_in_area}, '
          f'{t_ms:.1f} ms')
    for f in cat.frontiers + cat.rejected:
        vp = (f'viewpoint N {f.viewpoint_ne[0]:+6.1f} E '
              f'{f.viewpoint_ne[1]:+6.1f}' if f.viewpoint_ne else
              'no viewpoint')
        print(f'   cluster {f.cluster:3d} size {f.size:4d} centroid N '
              f'{f.centroid_ne[0]:+6.1f} E {f.centroid_ne[1]:+6.1f} | {vp}'
              f' | gain {f.gain:4d} | dist {f.path_dist_m:6.1f} m'
              f'{" | " + f.rejected if f.rejected else ""}')


t0 = time.perf_counter()
cat = detect_frontiers(count, var, res, origin, vehicle_ne=(0.0, 0.0),
                       explore_mask=world)
show('survey map v6, validation world', cat,
     1e3 * (time.perf_counter() - t0))

seen = np.hypot(cn, ce) <= 18.0
c2 = np.where(seen, count, 0)
v2 = np.where(seen, var, np.inf)
t0 = time.perf_counter()
cat = detect_frontiers(c2, v2, res, origin, vehicle_ne=(0.0, 0.0),
                       explore_mask=world)
show('early flight, 18 m footprint from take-off', cat,
     1e3 * (time.perf_counter() - t0))
