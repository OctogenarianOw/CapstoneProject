#!/usr/bin/env python3
"""Cells shown green (landing candidate) then red (refused) in a filmed run.

Replays the perception on every 10 s map snapshot of a mission, with the
colours of viz.classification_cloud, and lists the cells that were green
at some snapshot and red at the last one: which test refuses them, what
the truth says, how far the nearest truth obstacle is.
    python3 green_to_red.py RUN_DIR [RUN_DIR ...]
"""
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
from scipy.ndimage import distance_transform_edt
from uav_core.observation import OBSERVED, UNKNOWN, VAR_MAX
from uav_core.terrain_analysis import analyse
from uav_core.world_model import build_world, THRESHOLDS, WorldParams

TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_truth.npz'
t = np.load(TRUTH)
CENTRES = t['centres']
NED = -t['truth'].T
TL = analyse(NED, np.ones(NED.shape, np.int32), 1.0)
TRUTH_OK = ((TL['slope'] <= THRESHOLDS['max_slope_deg'])
            & (TL['roughness'] <= THRESHOLDS['max_roughness_m'])
            & (TL['step'] <= THRESHOLDS['max_step_m']))
D_OBST = distance_transform_edt(~(TL['step'] > THRESHOLDS['max_step_m']))


def colours(path, vehicle):
    m = np.load(path)
    res = float(m['resolution'])
    origin = (float(m['origin_north']), float(m['origin_east']))
    w = build_world(m['elevation'], m['variance'], m['count'], res, origin,
                    vehicle, WorldParams())
    st, lay, land = w.state, w.layers, w.landing
    seen = st != UNKNOWN
    uncertain = seen & ((st != OBSERVED) | ~lay['valid'])
    refused = seen & ~uncertain & ~land['cell_ok']
    cand = land['candidate']
    with np.errstate(invalid='ignore'):
        flags = {
            'slope': lay['slope'] > THRESHOLDS['max_slope_deg'],
            'roughness': lay['roughness'] > THRESHOLDS['max_roughness_m'],
            'step': lay['step'] > THRESHOLDS['max_step_m'],
            'variance': ~(m['variance'] <= VAR_MAX),
        }
    return cand, refused, flags, res, origin


def truth_ij(ne):
    return (int(np.argmin(np.abs(CENTRES - ne[0]))),
            int(np.argmin(np.abs(CENTRES - ne[1]))))


for run in map(Path, sys.argv[1:]):
    mission = sorted((run / 'missions').iterdir())[-1]
    cycles = [json.loads(x) for x in open(mission / 'cycles.jsonl')]
    snaps = sorted((mission / 'maps').glob('map_*.npz'),
                   key=lambda p: float(p.stem.split('_')[1]))
    ever_green = None
    first_green = None
    went_red_at = None
    for p in snaps:
        ts = float(p.stem.split('_')[1])
        upto = [c for c in cycles if c['t'] <= ts + 0.5] or cycles[:1]
        cand, refused, flags, res, origin = colours(
            p, upto[-1]['vehicle_ned'][:2])
        if ever_green is None:
            ever_green = np.zeros(cand.shape, bool)
            first_green = np.full(cand.shape, np.nan)
            went_red_at = np.full(cand.shape, np.nan)
        newly = cand & ~ever_green
        first_green[newly] = ts
        ever_green |= cand
        flip = ever_green & refused & np.isnan(went_red_at)
        went_red_at[flip] = ts
    final_red = ever_green & refused
    ii, jj = np.nonzero(final_red)
    why = Counter()
    truth = Counter()
    dist = []
    delay = []
    for i, j in zip(ii, jj):
        why['+'.join(k for k, f in flags.items() if f[i, j]) or 'footprint'] += 1
        ne = (origin[0] + (i + 0.5) * res, origin[1] + (j + 0.5) * res)
        ti, tj = truth_ij(ne)
        truth['truth landable' if TRUTH_OK[ti, tj] else 'truth unsafe'] += 1
        dist.append(D_OBST[ti, tj])
        delay.append(went_red_at[i, j] - first_green[i, j])
    print(f'== {run.name}: {len(snaps)} snapshots')
    print(f'  cells ever green: {int(ever_green.sum())}, '
          f'green then red at the end: {int(final_red.sum())}')
    print(f'  refused by: {dict(why)}')
    print(f'  truth: {dict(truth)}')
    if dist:
        print(f'  distance to nearest truth obstacle (m): median '
              f'{np.median(dist):.1f}, p90 {np.percentile(dist, 90):.1f}')
        print(f'  time green before red (s): median {np.median(delay):.0f}')
