#!/usr/bin/env python3
"""
Convert a mission map snapshot to the saved-map format of cloud_world.

Snapshots (mission_log.py) store origin_north and origin_east; the tools
of tools/ and experiments/ read origin_x and origin_y, as written by
cloud_world.save_map. Lets any snapshot be evaluated against the truth
with the existing tools.

    python3 snapshot_to_map.py SNAPSHOT.npz OUT.npz
"""

import sys

import numpy as np

s = np.load(sys.argv[1])
np.savez(sys.argv[2], elevation=s['elevation'], variance=s['variance'],
         count=s['count'], resolution=s['resolution'],
         origin_x=s['origin_north'], origin_y=s['origin_east'])
print(f'written {sys.argv[2]}')
