#!/usr/bin/env python3
"""
I4 per cell against I4 per touchdown point, replayed offline.

I4: a zone of the registry has been observed from at least two distinct
passes. The registry counts passes around one touchdown point per zone
(an entry within r_in = 0.8 sensor radius after leaving the sensor
radius), and the first W3 flight (demos/2026-10-22_10-49_w3) never
confirmed the road: its point fell 50 m west, flown near once. The
variant measured here applies the same rule to every cell: a cell has
been passed when the aircraft entered within r_in of it after being
beyond the sensor radius; the confirmable part of a zone is its
landable cells passed at least twice, kept where they hold a whole
footprint (the min-zone rule).

For each mission, from its own log (track at 1 Hz in cycles.jsonl, as
the registry samples it) and its map snapshots (every 10 s), the chain
of the manager is run again on each snapshot (observation states, terrain
layers, landing detection on OBSERVED cells) and compared:

  point   what the mission did: non-provisional zones CONFIRMED in the
          log, their cells taken as the detected components holding
          their touchdown
  cell    the variant: twice-passed landable cells in components that
          hold a whole footprint

Reported: time of the first confirmed zone, confirmed landable area at
the last snapshot, and, against the truth, confirmed cells whose real
slope or step is beyond twice the threshold (severe, as validate_slz),
the S8 barrier. Also the candidate cells of the last map that are
severe, whatever their confirmation.

    python3 replay.py WORLD MISSION_DIR [MISSION_DIR ...]
    WORLD: w1, w2 or w3 (selects the truth)
"""

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src' / 'uav_core'))
from uav_core.observation import classify  # noqa: E402
from uav_core.slz_detection import footprint_ok, label_zones  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402
from uav_core.world_model import detect_landing, THRESHOLDS  # noqa: E402

TRUTH = {'w1': 'validation_truth.npz', 'w2': 'w2_truth.npz',
         'w3': 'w3_truth.npz'}
SENSOR_RADIUS_M = 18.0
R_IN = 0.8 * SENSOR_RADIUS_M
FOOTPRINT_M = 1.5


def load_jsonl(path):
    """Records of one JSON lines file."""
    return [json.loads(x) for x in path.read_text().splitlines()
            if x.strip()]


def truth_severe(world, shape, res, origin):
    """Severe mask in the map grid: real slope > 12 deg or step > 0.48 m."""
    t = np.load(ROOT / 'worlds' / TRUTH[world])
    truth_ned = -t['truth'].T
    layers = analyse(truth_ned, np.ones(truth_ned.shape, np.int32), res)
    sev = ((layers['slope'] > 2 * THRESHOLDS['max_slope_deg'])
           | (layers['step'] > 2 * THRESHOLDS['max_step_m']))
    n = truth_ned.shape[0]
    half = n * res / 2.0
    i0 = int(round((-half + res / 2.0 - origin[0]) / res))
    j0 = int(round((-half + res / 2.0 - origin[1]) / res))
    out = np.zeros(shape, dtype=bool)
    out[i0:i0 + n, j0:j0 + n] = sev
    return out


class PassMap:
    """Passes per cell over a track, the registry's rule on every cell."""

    def __init__(self, shape, res, origin):
        i, j = np.indices(shape)
        self.n = origin[0] + (i + 0.5) * res
        self.e = origin[1] + (j + 0.5) * res
        self.inside = np.zeros(shape, dtype=bool)
        self.passes = np.zeros(shape, dtype=np.int16)

    def add(self, ne):
        d = np.hypot(self.n - ne[0], self.e - ne[1])
        enter = ~self.inside & (d < R_IN)
        self.passes[enter] += 1
        self.inside |= enter
        self.inside &= ~(d > SENSOR_RADIUS_M)


def confirmed_cells_by_point(landing, zones, res, origin):
    """Cells of the detected components holding a CONFIRMED touchdown."""
    labels = landing['labels']
    out = np.zeros(labels.shape, dtype=bool)
    for z in zones:
        if z['provisional'] or z['state'] != 'CONFIRMED':
            continue
        i = int((z['touchdown_ne'][0] - origin[0]) / res)
        j = int((z['touchdown_ne'][1] - origin[1]) / res)
        if 0 <= i < labels.shape[0] and 0 <= j < labels.shape[1] \
                and labels[i, j]:
            out |= labels == labels[i, j]
    return out


def confirmed_cells_by_cell(landing, passes):
    """Twice-passed landable cells in components with a whole footprint."""
    twice = landing['candidate'] & (passes >= 2)
    core = footprint_ok(twice, landing['footprint_offsets'])
    labels, n = label_zones(twice)
    keep = np.zeros(twice.shape, dtype=bool)
    for k in range(1, n + 1):
        m = labels == k
        if core[m].any():
            keep |= m
    return keep


def mission(world, mdir):
    """Replay one mission, return its row."""
    cycles = load_jsonl(mdir / 'cycles.jsonl')
    snaps = sorted((mdir / 'maps').glob('map_*.npz'))
    first = {'point': None, 'cell': None}
    ci = 0
    pm = sev = None
    worst = {'point': 0, 'cell': 0}
    for snap in snaps:
        m = np.load(snap)
        t = float(snap.stem.split('_')[1])
        res = float(m['resolution'])
        origin = (float(m['origin_north']), float(m['origin_east']))
        if pm is None:
            pm = PassMap(m['count'].shape, res, origin)
            sev = truth_severe(world, m['count'].shape, res, origin)
        zones = []
        while ci < len(cycles) and cycles[ci]['t'] <= t:
            pm.add(cycles[ci]['vehicle_ned'][:2])
            zones = cycles[ci]['zones']
            ci += 1
        state = classify(m['count'], m['variance'])
        layers = analyse(m['elevation'], m['count'], res)
        landing = detect_landing(layers, m['elevation'], m['variance'],
                                 state, res, origin, FOOTPRINT_M)
        conf = {'point': confirmed_cells_by_point(landing, zones, res,
                                                  origin),
                'cell': confirmed_cells_by_cell(landing, pm.passes)}
        for k, c in conf.items():
            if c.any() and first[k] is None:
                first[k] = t
            worst[k] = max(worst[k], int((c & sev).sum()))
    return {'run': mdir.parents[1].name if mdir.parents[1].name != 'missions'
            else mdir.name,
            'first_point': first['point'], 'first_cell': first['cell'],
            'area_point': int(conf['point'].sum()),
            'area_cell': int(conf['cell'].sum()),
            'severe_point': worst['point'], 'severe_cell': worst['cell'],
            'severe_candidates_last': int((landing['candidate'] & sev).sum()),
            'end_t': t}


def main():
    world = sys.argv[1]
    rows = [mission(world, Path(p)) for p in sys.argv[2:]]
    print(f'{"run":<26} {"first confirmed, s":>19} {"area at the end, m2":>20}'
          f' {"severe confirmed":>17} {"severe candidates":>18}')
    print(f'{"":<26} {"point":>9} {"cell":>9} {"point":>10} {"cell":>9}'
          f' {"point":>8} {"cell":>8} {"last map":>18}')
    for r in rows:
        fp = '-' if r['first_point'] is None else f'{r["first_point"]:.0f}'
        fc = '-' if r['first_cell'] is None else f'{r["first_cell"]:.0f}'
        print(f'{r["run"]:<26} {fp:>9} {fc:>9} {r["area_point"]:>10}'
              f' {r["area_cell"]:>9} {r["severe_point"]:>8}'
              f' {r["severe_cell"]:>8} {r["severe_candidates_last"]:>18}')
    n = len(rows)
    for k in ('point', 'cell'):
        conf = [r for r in rows if r[f'first_{k}'] is not None]
        area = np.median([r[f'area_{k}'] for r in rows])
        print(f'{k:>5}: a zone confirmed in {len(conf)} of {n} missions, '
              f'median first {np.median([r[f"first_{k}"] for r in conf]) if conf else float("nan"):.0f} s, '
              f'median area at the end {area:.0f} m2, '
              f'severe confirmed cells in {sum(r[f"severe_{k}"] > 0 for r in rows)} missions '
              f'(max {max(r[f"severe_{k}"] for r in rows)})')


if __name__ == '__main__':
    main()
