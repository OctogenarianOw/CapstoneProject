#!/usr/bin/env python3
"""
Why a confirmed zone loses its landing cells: the most probable cause of
the pass-count demotions of this campaign, reconstructed offline.

The registry cannot be replayed cycle by cycle (the map is saved every
10 s), but the two things a landing cell depends on can be rebuilt:
  - the passes per cell, exactly: the PassGrid of slz_registry fed the
    vehicle position of every cycle (cycles.jsonl), as in flight;
  - the landable cells, on the map snapshots that bracket the demotion
    (the last one at or before it, the first one after it), run through
    build_world, the perception code that ran in flight.
A landing cell c needs every cell f of its footprint to be a candidate
with 2 passes, and a candidate needs every cell g of its own footprint
to pass the cell test (classify_cells: analysis window observed,
slope <= 6 deg, roughness <= 0.05 m, step <= 0.24 m, cell OBSERVED with
variance <= VAR_MAX). Passes per cell only grow, so a landing cell can
only be lost (a) to a split, still landing but in another zone, or (b)
because some g within two footprints of it went from accepted to
refused. For each demotion this script lists the lost landing cells,
the newly refused g that block them, which test each g fails, how its
layers changed between the snapshots, and where it lies: on the zone's
border or inside, how far from the aircraft.

Approximation, stated: the snapshot before may predate the
confirmation by up to 10 s and the one after may follow the demotion by
up to 10 s; both gaps are printed, and demotions whose snapshot before
holds no landing cell for the zone are counted apart.

    python3 demotion_cause.py [CAMPAIGN_DIR]   # ROS and the workspace sourced
"""

from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

from scipy.ndimage import distance_transform_edt
from uav_core.observation import OBSERVED, VAR_MAX
from uav_core.terrain_analysis import analyse
from uav_core.slz_detection import footprint_ok
from uav_core.slz_registry import footprint_min, PassGrid, SlzRegistry
from uav_core.world_model import build_world, THRESHOLDS, WorldParams

HERE = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).parent
REG = SlzRegistry()
TRUTH = Path.home() / 'ros2_ws/droneAfter/worlds/w3_truth.npz'


def truth_layers():
    """Truth roughness and distance to the nearest truth obstacle."""
    t = np.load(TRUTH)
    ned = -t['truth'].T
    lay = analyse(ned, np.ones(ned.shape, np.int32), 1.0)
    # An obstacle: a truth cell with a step a landing would refuse.
    obstacle = lay['step'] > THRESHOLDS['max_step_m']
    return t['centres'], lay['roughness'], distance_transform_edt(~obstacle)


CENTRES, TRUTH_ROUGH, TRUTH_D_OBST = truth_layers()


def truth_at(ne):
    i = int(np.argmin(np.abs(CENTRES - ne[0])))
    j = int(np.argmin(np.abs(CENTRES - ne[1])))
    return float(TRUTH_ROUGH[i, j]), float(TRUTH_D_OBST[i, j])


def load(path):
    return [json.loads(x) for x in path.open() if x.strip()]


def snapshots(mission):
    snaps = sorted((mission / 'maps').glob('map_*.npz'),
                   key=lambda p: float(p.stem.split('_')[1]))
    return [(float(p.stem.split('_')[1]), p) for p in snaps]


def rebuild(path, t, cycles):
    """Perception and passes of one snapshot, as the registry saw them."""
    m = np.load(path)
    res = float(m['resolution'])
    origin = (float(m['origin_north']), float(m['origin_east']))
    upto = [c for c in cycles if c['t'] <= t + 0.5]
    vehicle = upto[-1]['vehicle_ned'][:2]
    w = build_world(m['elevation'], m['variance'], m['count'], res, origin,
                    vehicle, WorldParams())
    land = w.landing
    grid = PassGrid(land['candidate'].shape, res, origin,
                    REG.pass_hysteresis * REG.sensor_radius_m,
                    REG.sensor_radius_m)
    for c in upto:
        grid.add(c['vehicle_ned'][:2])
    cand = land['candidate']
    fp = footprint_min(np.where(cand, grid.passes, 0),
                       land['footprint_offsets'])
    lay = w.layers
    with np.errstate(invalid='ignore'):
        flags = {
            'window not observed': ~lay['valid'],
            'slope': lay['slope'] > THRESHOLDS['max_slope_deg'],
            'roughness': lay['roughness'] > THRESHOLDS['max_roughness_m'],
            'step': lay['step'] > THRESHOLDS['max_step_m'],
            'variance': (w.state != OBSERVED) | ~(m['variance'] <= VAR_MAX),
        }
    return {'res': res, 'origin': origin, 'vehicle': vehicle,
            'cell_ok': land['cell_ok'], 'cand': cand,
            'labels': land['labels'], 'landing': cand & (fp >= 2),
            'offsets': land['footprint_offsets'], 'flags': flags,
            'layers': lay, 'variance': m['variance'], 'count': m['count'],
            'elevation': m['elevation']}


def ij(ne, s):
    return (int((ne[0] - s['origin'][0]) / s['res']),
            int((ne[1] - s['origin'][1]) / s['res']))


def ne_of(c, s):
    return (s['origin'][0] + (c[0] + 0.5) * s['res'],
            s['origin'][1] + (c[1] + 0.5) * s['res'])


def main():
    totals = Counter()
    kinds = Counter()
    place = Counter()
    changes = []
    n = n_empty = 0
    for run in sorted((HERE / 'runs').iterdir()):
        if not run.is_dir() or run.name.startswith('.'):
            continue
        mission = next((run / 'missions').iterdir())
        events = load(mission / 'events.jsonl')
        cycles = load(mission / 'cycles.jsonl')
        snaps = snapshots(mission)
        for e in events:
            if not (e['kind'] == 'zone' and e['old'] == 'CONFIRMED'
                    and e['new'] == 'CANDIDATE' and 'pass' in e['reason']):
                continue
            n += 1
            td = e['t']
            before = [s for s in snaps if s[0] <= td]
            after = [s for s in snaps if s[0] > td]
            if not before or not after:
                continue
            (tb, pb), (ta, pa) = before[-1], after[0]
            prev = [c for c in cycles if c['t'] < td - 0.5][-1]
            zone = next(z for z in prev['zones'] if z['id'] == e['zone_id'])
            sb = rebuild(pb, tb, cycles)
            sa = rebuild(pa, ta, cycles)
            ti = ij(zone['touchdown_ne'], sb)
            lab_b = sb['labels'][ti]
            if lab_b == 0:
                n_empty += 1
                print(f'{run.name} {td:6.1f} zone {e["zone_id"]}: touchdown '
                      f'not a candidate in the snapshot before ({td - tb:.1f}'
                      f' s earlier)')
                continue
            zone_b = sb['labels'] == lab_b
            land_b = np.argwhere(zone_b & sb['landing'])
            if not len(land_b):
                n_empty += 1
                print(f'{run.name} {td:6.1f} zone {e["zone_id"]}: no landing '
                      f'cell in the snapshot before ({td - tb:.1f} s earlier)')
                continue
            lab_a = sa['labels'][ij(zone['touchdown_ne'], sa)]
            lost, split = [], 0
            for c in map(tuple, land_b):
                if sa['landing'][c] and sa['labels'][c] == lab_a and lab_a:
                    continue
                if sa['landing'][c]:
                    split += 1
                else:
                    lost.append(c)
            # Newly refused cells within two footprints of a lost cell.
            reach = set()
            for c in lost:
                for di, dj in sb['offsets']:
                    for ei, ej in sb['offsets']:
                        reach.add((c[0] + di + ei, c[1] + dj + ej))
            newly = [g for g in reach
                     if 0 <= g[0] < sb['cell_ok'].shape[0]
                     and 0 <= g[1] < sb['cell_ok'].shape[1]
                     and sb['cell_ok'][g] and not sa['cell_ok'][g]]
            why = Counter()
            for g in newly:
                failed = tuple(k for k, f in sa['flags'].items() if f[g])
                why[failed] += 1
                totals[failed] += 1
                border = any(not sb['cand'][g[0] + di, g[1] + dj]
                             for di in (-1, 0, 1) for dj in (-1, 0, 1)
                             if (di, dj) != (0, 0))
                place['zone border' if border else 'zone inside'] += 1
                tr, dob = truth_at(ne_of(g, sa))
                win = (slice(g[0] - 1, g[0] + 2), slice(g[1] - 1, g[1] + 2))
                changes.append({
                    'truth_rough': tr, 'd_obstacle_m': dob,
                    'count_gain_window': float(np.max(
                        sa['count'][win].astype(float) / np.maximum(
                            sb['count'][win].astype(float), 1.0))),
                    'height_shift_window_m': float(np.nanmax(np.abs(
                        sa['elevation'][win] - sb['elevation'][win]))),
                    'd_vehicle_m': math.dist(ne_of(g, sa), sa['vehicle']),
                    'var_ratio': float(sa['variance'][g] /
                                       max(sb['variance'][g], 1e-9)),
                    'step_b': float(sb['layers']['step'][g]),
                    'step_a': float(sa['layers']['step'][g]),
                    'rough_b': float(sb['layers']['roughness'][g]),
                    'rough_a': float(sa['layers']['roughness'][g]),
                    'slope_b': float(sb['layers']['slope'][g]),
                    'slope_a': float(sa['layers']['slope'][g])})
            kind = ('split' if split and not lost else
                    'newly refused cells' if newly else
                    'lost, no newly refused cell found' if lost else
                    'no loss between the snapshots')
            kinds[kind] += 1
            top = ', '.join(f'{"+".join(k) or "none"} {v}'
                            for k, v in why.most_common(3))
            print(f'{run.name} {td:6.1f} zone {e["zone_id"]}: snapshots '
                  f'-{td - tb:.1f} s / +{ta - td:.1f} s, landing cells '
                  f'{len(land_b)}, lost {len(lost)}, split {split}, newly '
                  f'refused {len(newly)} ({top}); {kind}')

    print(f'\n{n} pass-count demotions, {n_empty} without landing cells in '
          f'the snapshot before')
    print('per demotion:', dict(kinds))
    print('newly refused cells by failed test:',
          {'+'.join(k): v for k, v in totals.most_common()})
    print('where they lie:', dict(place))
    if changes:
        def med(k):
            return float(np.nanmedian([c[k] for c in changes]))
        print(f'median over the newly refused cells: distance to the '
              f'aircraft {med("d_vehicle_m"):.1f} m; variance x'
              f'{med("var_ratio"):.2f}; step {med("step_b"):.3f} -> '
              f'{med("step_a"):.3f} m; roughness {med("rough_b"):.3f} -> '
              f'{med("rough_a"):.3f} m; slope {med("slope_b"):.1f} -> '
              f'{med("slope_a"):.1f} deg')
        print(f'truth under them: roughness {med("truth_rough"):.3f} m '
              f'(over 0.05 m in '
              f'{sum(c["truth_rough"] > 0.05 for c in changes)} of '
              f'{len(changes)}); distance to the nearest truth obstacle '
              f'{med("d_obstacle_m"):.1f} m (within 3 m: '
              f'{sum(c["d_obstacle_m"] <= 3 for c in changes)}); largest '
              f'gain of observations in their 3 x 3 window x'
              f'{med("count_gain_window"):.2f}, largest height shift there '
              f'{med("height_shift_window_m"):.2f} m')


if __name__ == '__main__':
    main()
