#!/usr/bin/env python3
"""
I4 per cell in flight: before (passes around one touchdown point) and
after (passes per cell), on W3 and W1.

Per mission, from its log: end reason, I1 violations and the smallest
I1 margin, coverage of the area, energy left at the landing against the
reserve, the zones confirmed besides the take-off point (where they lie,
by the truth label at their touchdown) and the first time one was.

Safety, on every map snapshot (every 10 s), the cells C1 may target
that are severe in the truth (real slope over 12 deg or step over
0.48 m, as tools/validate_slz.py): before, every landable cell of the
confirmed zones, what the return field seeded; after, those whose whole
footprint was passed twice, the registry's landing cells, recomputed
with the branch's own PassGrid and footprint_min from the track. And
the landing cell: severe or not, landable in the truth or not.

    python3 analyse.py           # prints the table and the summary
"""

import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(Path.home() / 'uav_i4' / 'src' / 'uav_core'))
from uav_core.observation import classify  # noqa: E402
from uav_core.slz_detection import detect  # noqa: E402
from uav_core.slz_registry import footprint_min, PassGrid  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402
from uav_core.world_model import detect_landing, THRESHOLDS  # noqa: E402

TRUTH = {'w3': 'w3_truth.npz', 'w1': 'validation_truth.npz'}


def load(path):
    """Records of one JSON lines file."""
    return [json.loads(x) for x in path.read_text().splitlines()
            if x.strip()]


class Truth:
    """Truth of a world laid on the map grid of a mission."""

    def __init__(self, world, shape, res, origin):
        t = np.load(ROOT / 'worlds' / TRUTH[world])
        ned = -t['truth'].T
        zone = t['zone'].T
        self.names = [str(s) for s in t['zone_names']]
        lay = analyse(ned, np.ones(ned.shape, np.int32), res)
        severe = ((lay['slope'] > 2 * THRESHOLDS['max_slope_deg'])
                  | (lay['step'] > 2 * THRESHOLDS['max_step_m']))
        safe = detect(lay, ned, res, (0.0, 0.0), footprint_m=1.5,
                      **THRESHOLDS)['candidate']
        n = ned.shape[0]
        half = n * res / 2.0
        self.i0 = int(round((-half + res / 2.0 - origin[0]) / res))
        self.j0 = int(round((-half + res / 2.0 - origin[1]) / res))
        self.severe = np.zeros(shape, dtype=bool)
        self.safe = np.zeros(shape, dtype=bool)
        self.zone = np.full(shape, -1, dtype=int)
        box = (slice(self.i0, self.i0 + n), slice(self.j0, self.j0 + n))
        self.severe[box], self.safe[box], self.zone[box] = severe, safe, zone
        self.res, self.origin = res, origin

    def cell(self, ne):
        """Map cell of a point."""
        return (int((ne[0] - self.origin[0]) / self.res),
                int((ne[1] - self.origin[1]) / self.res))

    def label(self, ne):
        """Truth zone name at a point."""
        k = self.zone[self.cell(ne)]
        return self.names[k] if k >= 0 else 'outside'


def targets(landing, zones, passes, after, truth):
    """Cells C1 may target on one snapshot, under either rule."""
    labels = landing['labels']
    out = np.zeros(labels.shape, dtype=bool)
    for z in zones:
        if z['provisional'] or z['state'] != 'CONFIRMED':
            continue
        i, j = truth.cell(z['touchdown_ne'])
        if labels[i, j]:
            out |= labels == labels[i, j]
    if after:
        cand = landing['candidate']
        fp = footprint_min(np.where(cand, passes, 0),
                           landing['footprint_offsets'])
        out &= fp >= 2
    return out


def mission(run, world=None, variant=None):
    """One row of the table; world and variant from the run name."""
    if world is None:
        world, variant = run.name.split('_')[:2]
    mdir = next((run / 'missions').iterdir())
    cycles = load(mdir / 'cycles.jsonl')
    config = json.load(open(mdir / 'config.json'))
    events = load(mdir / 'events.jsonl')
    last = cycles[-1]
    capacity = last['remaining_j'] + last['consumed_j']
    reserve = config['reserve_fraction'] * capacity
    margins = [c['i1_margin_j'] for c in cycles
               if isinstance(c['i1_margin_j'], (int, float))]
    landed = [c for c in cycles if c['state'] == 'LANDED'][0]
    end = [e['why'] for e in events if e.get('kind') == 'state'
           and e.get('new') == 'LANDED'][0]
    grid = truth = None
    ci, worst, first = 0, 0, None
    conf_at = {}
    for snap in sorted((mdir / 'maps').glob('map_*.npz')):
        m = np.load(snap)
        t = float(snap.stem.split('_')[1])
        res = float(m['resolution'])
        origin = (float(m['origin_north']), float(m['origin_east']))
        if grid is None:
            grid = PassGrid(m['count'].shape, res, origin, 14.4, 18.0)
            truth = Truth(world, m['count'].shape, res, origin)
        zones = []
        while ci < len(cycles) and cycles[ci]['t'] <= t:
            grid.add(cycles[ci]['vehicle_ned'][:2])
            zones = cycles[ci]['zones']
            ci += 1
        landing = detect_landing(analyse(m['elevation'], m['count'], res),
                                 m['elevation'], m['variance'],
                                 classify(m['count'], m['variance']), res,
                                 origin, 1.5)
        tg = targets(landing, zones, grid.passes, variant == 'after', truth)
        worst = max(worst, int((tg & truth.severe).sum()))
    for c in cycles:
        for z in c['zones']:
            if z['provisional'] or z['state'] != 'CONFIRMED':
                continue
            if first is None:
                first = c['t']
            where = truth.label(z['touchdown_ne'])
            conf_at.setdefault(where, c['t'])
    li = truth.cell(landed['vehicle_ned'][:2])
    return {
        'run': run.name, 'world': world, 'variant': variant, 'end': end,
        'violations': last['violations'],
        'min_margin': min(margins) if margins else float('nan'),
        'coverage': 100.0 * last['seen_in_area']
        / (last['seen_in_area'] + last['unknown_in_area']),
        'left_over_reserve': landed['remaining_j'] - reserve,
        'first': first, 'confirmed': conf_at, 'severe_targets': worst,
        'landing': truth.label(landed['vehicle_ned'][:2]),
        'landing_safe': bool(truth.safe[li]),
        'landing_severe': bool(truth.severe[li]),
        'clock_steps': sum(1 for x in (run / 'clock.log').read_text()
                           .splitlines() if x.startswith('step')),
    }


def main():
    runs = sorted(p for p in (HERE / 'runs').iterdir()
                  if p.is_dir() and not p.name.startswith('.'))
    rows = [mission(r) for r in runs]
    print(f'{"run":<14} {"end":<15} {"viol":>4} {"min margin":>10} '
          f'{"cover":>6} {"left-R":>7} {"1st conf":>8} {"severe tg":>9} '
          f'{"landed on":<16} {"clock":>5}  confirmed zones (first time)')
    for r in rows:
        conf = ', '.join(f'{k} {v:.0f}s' for k, v in
                         sorted(r['confirmed'].items(), key=lambda x: x[1]))
        first = '-' if r['first'] is None else f'{r["first"]:.0f}'
        # Severe is the safety reading; "edge" marks a cell flat in the
        # truth whose footprint touches a cell refused by its window
        land = r['landing'] + (' SEVERE' if r['landing_severe'] else
                               '' if r['landing_safe'] else ' edge')
        print(f'{r["run"]:<14} {r["end"]:<15} {r["violations"]:>4} '
              f'{r["min_margin"]:>10.0f} {r["coverage"]:>5.1f}% '
              f'{r["left_over_reserve"]:>7.0f} {first:>8} '
              f'{r["severe_targets"]:>9} {land:<16} {r["clock_steps"]:>5}'
              f'  {conf}')
    print()
    for world in ('w3', 'w1'):
        for variant in ('before', 'after'):
            sel = [r for r in rows if r['world'] == world
                   and r['variant'] == variant]
            if not sel:
                continue
            other = sum(any(k != 'yard' for k in r['confirmed'])
                        for r in sel) if world == 'w3' else None
            print(f'{world} {variant:<6} n={len(sel)}: violations '
                  f'{sum(r["violations"] for r in sel)}, severe targets '
                  f'{sum(r["severe_targets"] for r in sel)}, landings on a '
                  f'severe cell {sum(r["landing_severe"] for r in sel)}, '
                  'coverage '
                  f'median {np.median([r["coverage"] for r in sel]):.1f} %'
                  + (f', a zone besides the yard confirmed in {other}'
                     if other is not None else ''))


if __name__ == '__main__':
    main()
