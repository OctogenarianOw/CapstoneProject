#!/usr/bin/env python3
"""
Pit calibration: is a pit hidden from afar and revealed from close?

For every map snapshot of every flight (pits_drive.py), and every pit of
worlds/pits_truth.npz, the distance at which the aircraft had passed the
pit so far (closest horizontal approach in track.csv up to the snapshot)
against what the map makes of the pit:

  step_max    largest step layer (S7) in the 3 x 3 block around the
              truth pit cell, the quantity the 0.24 m threshold reads.
              The block, not the cell, because P5 measures a one-cell
              offset between map and truth on gradients
  depth_seen  median plate top minus the lowest observed cell of the
              block, in metres, against the true depth
  block_ok    every cell of the block passes the cell test (S8) on
              OBSERVED cells, as the manager classifies them
  confirm     the plate holds a whole footprint element of landable
              cells, the min-zone rule of the registry: the plate could
              be confirmed

Criterion, fixed on 21 Oct before any flight and reported to Dorian: a
depth works for W3 if every pit passed at 10 m or more so far has
step_max < 0.24 m (hidden: the classifier accepts it) and every pit
passed within 5 m has step_max > 0.48 m (revealed: refused, and past
the severe limit of S8). A block with an unobserved cell counts as not
hidden, since I2 refuses it. Among the depths that meet the criterion in
every pit of every flight, W3 takes the one that stays hidden down to
the smallest distance, the smaller depth on a tie. Reason: the registry
confirms a zone after two passes within 14.4 m of its touchdown point,
and the pits spread over the 16 m of the pad, so a pass at 14 m from
the centre already comes within about 8.5 m of the nearest pits. What
W3 needs is the widest band in which the trap can be confirmed unseen,
not the least extreme hole. Rule revised on 21 Oct, before any flight,
from "the smallest depth that meets the criterion". If no depth meets
the criterion, the deception by pits does not work in this simulator,
and W3 is not built on it.

    python3 analyse_pits.py RUN_DIR [RUN_DIR ...]
    python3 analyse_pits.py --truth     the exact truth, then the truth
                                        with filled pits, through the
                                        same chain: checks the indexing
"""

import csv
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))

from uav_core.observation import classify, OBSERVED, VAR_MAX  # noqa: E402
from uav_core.slz_detection import detect, footprint_ok  # noqa: E402
from uav_core.terrain_analysis import analyse  # noqa: E402
import validate_slz as v  # noqa: E402

TRUTH = ROOT / 'worlds' / 'pits_truth.npz'
HIDDEN_FROM_M = 10.0
REVEALED_WITHIN_M = 5.0
HIDDEN_STEP_M = 0.24
REVEALED_STEP_M = 0.48


def load_truth():
    """Pits and plate cells as NED (north, east) centres, per plate."""
    t = np.load(TRUTH)
    c = t['centres']
    names = [str(s) for s in t['zone_names']]
    zone, plate = t['zone'], t['plate']
    pits, cells = [], {}
    for ix, iy in zip(*np.nonzero(plate >= 0)):
        p = int(plate[ix, iy])
        ne = (float(c[iy]), float(c[ix]))
        cells.setdefault(p, []).append((ne, names[zone[ix, iy]]))
        if names[zone[ix, iy]] == 'pit':
            pits.append((p, ne))
    return t, pits, cells


def index(ne, origin, res):
    """Map cell (i north, j east) holding a NED point."""
    return (int(math.floor((ne[0] - origin[0]) / res)),
            int(math.floor((ne[1] - origin[1]) / res)))


def judge(snapshot, pits, cells):
    """Per pit and per plate readings of one map snapshot."""
    m = np.load(snapshot)
    elev, var, count = m['elevation'], m['variance'], m['count']
    res = float(m['resolution'])
    # cloud_world writes origin_x, origin_y (north, east: elevation_grid),
    # the manager's snapshots origin_north, origin_east.
    keys = (('origin_north', 'origin_east') if 'origin_north' in m.files
            else ('origin_x', 'origin_y'))
    origin = (float(m[keys[0]]), float(m[keys[1]]))
    state = classify(count, var)
    layers = analyse(elev, count, res)
    found = detect(layers, elev, res, origin, footprint_m=1.5,
                   variance=np.where(state == OBSERVED, var, np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    core = footprint_ok(found['candidate'], found['footprint_offsets'])
    up = -elev.astype(float)
    plates = {}
    for p, plate_cells in cells.items():
        idx = [index(ne, origin, res) for ne, _ in plate_cells]
        tops = [up[i, j] for (i, j), (_, z) in zip(idx, plate_cells)
                if z == 'top' and count[i, j] > 0]
        plates[p] = {
            'top': float(np.median(tops)) if tops else math.nan,
            'landable': int(sum(found['candidate'][i, j] for i, j in idx)),
            'confirm': bool(any(core[i, j] for i, j in idx)),
        }
    rows = []
    for p, ne in pits:
        i, j = index(ne, origin, res)
        block = np.s_[i - 1:i + 2, j - 1:j + 2]
        steps = layers['step'][block]
        seen = count[block] > 0
        unobserved = bool((state[block] != OBSERVED).any())
        rows.append({
            'plate': p, 'pit_n': ne[0], 'pit_e': ne[1],
            'step_max_m': (float(np.nanmax(steps))
                           if np.isfinite(steps).any() else math.nan),
            'depth_seen_m': (plates[p]['top'] - float(up[block][seen].min())
                             if seen.any() else math.nan),
            'pit_count': int(count[i, j]),
            'block_unobserved': unobserved,
            'block_ok': bool(found['cell_ok'][block].all()),
            'plate_landable': plates[p]['landable'],
            'plate_confirm': plates[p]['confirm'],
        })
    return rows


def read_csv(path):
    """Rows of a csv file as dicts."""
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def flight_rows(run, pits, cells, depths):
    """Every pit at every snapshot of one flight, with its distance."""
    track = np.array([[float(r['t']), float(r['north']), float(r['east'])]
                      for r in read_csv(run / 'track.csv')])
    snaps = [(int(r['phase'][4:]), float(r['t']))
             for r in read_csv(run / 'phases.csv')
             if r['phase'].startswith('snap')]
    out = []
    for line, t_snap in snaps:
        upto = track[track[:, 0] <= t_snap]
        for row in judge(run / f'snap_line{line}.npz', pits, cells):
            d = np.hypot(upto[:, 1] - row['pit_n'], upto[:, 2] - row['pit_e'])
            row.update(run=run.name, line=line,
                       depth_m=float(depths[row['plate']]),
                       min_dist_m=float(d.min()))
            out.append(row)
    return out


def hidden(r):
    """The classifier would accept the pit."""
    return (not r['block_unobserved'] and np.isfinite(r['step_max_m'])
            and r['step_max_m'] < HIDDEN_STEP_M)


def hidden_down_to(rows):
    """Smallest distance from which every pit passed so far is hidden.

    The distance d such that every reading taken with the pit passed at
    d or more is hidden; inf if the farthest reading is not.
    """
    out = math.inf
    for r in sorted(rows, key=lambda r: -r['min_dist_m']):
        if not hidden(r):
            break
        out = r['min_dist_m']
    return out


def revealed(r):
    """The pit is refused past the severe limit."""
    return np.isfinite(r['step_max_m']) and r['step_max_m'] > REVEALED_STEP_M


def report(rows, depths):
    """Criterion per depth, then the depth W3 takes."""
    print(f'{"depth":>5} {"far pits (>= 10 m)":>20} {"hidden":>7} '
          f'{"near pits (<= 5 m)":>20} {"revealed":>8}  meets  '
          f'hidden down to')
    works = []
    for depth in depths:
        rs = [r for r in rows if r['depth_m'] == depth]
        far = [r for r in rs if r['min_dist_m'] >= HIDDEN_FROM_M]
        near = [r for r in rs if r['min_dist_m'] <= REVEALED_WITHIN_M]
        nh, nr = sum(map(hidden, far)), sum(map(revealed, near))
        ok = bool(far and near and nh == len(far) and nr == len(near))
        down_to = hidden_down_to(rs)
        if ok:
            works.append((down_to, depth))
        print(f'{depth:5.1f} {len(far):20d} {nh:7d} {len(near):20d} '
              f'{nr:8d}  {"yes" if ok else "no":5s}  {down_to:5.1f} m')
    print('\nstep_max and depth seen against the distance passed, median '
          'over pits and flights')
    bins = [(0, 2), (2, 5), (5, 8), (8, 10), (10, 13), (13, 17), (17, 99)]
    print(f'{"depth":>5} ' + ' '.join(f'{f"{a}-{b} m":>16}' for a, b in bins))
    for depth in depths:
        cells = []
        for a, b in bins:
            rs = [r for r in rows if r['depth_m'] == depth
                  and a <= r['min_dist_m'] < b]
            s = [r['step_max_m'] for r in rs if np.isfinite(r['step_max_m'])]
            ds = [r['depth_seen_m'] for r in rs
                  if np.isfinite(r['depth_seen_m'])]
            cells.append(f'{np.median(s):.2f}/{np.median(ds):.2f} n{len(rs)}'
                         if s and ds else f'{"n" + str(len(rs)):>16}')
        print(f'{depth:5.1f} ' + ' '.join(f'{c:>16}' for c in cells))
    print('\nplate confirmable (whole footprint) after each line, per flight')
    for run in sorted({r['run'] for r in rows}):
        for depth in depths:
            seq = []
            for line in sorted({r['line'] for r in rows if r['run'] == run}):
                rs = [r for r in rows if r['run'] == run
                      and r['line'] == line and r['depth_m'] == depth]
                seq.append('C' if rs and rs[0]['plate_confirm'] else '.')
            print(f'{run:<12} depth {depth:.1f}  {"".join(seq)}')
    print('\nW3 depth: ' + (f'{min(works)[1]:.1f} m, hidden down to '
                            f'{min(works)[0]:.1f} m, the lowest among the '
                            'depths that meet the criterion' if works else
                            'none meets the criterion'))


def truth_check(t, pits, cells):
    """The exact truth and the filled truth through the same chain."""
    tmp = Path('/tmp/pits_truth_as_map.npz')
    h = t['truth']
    filled = np.where(t['zone'] == 2,
                      np.array(t['plate_depths'])[np.maximum(t['plate'], 0)],
                      h)
    half = len(t['centres']) * float(t['resolution']) / 2.0
    for label, height in (('exact truth', h), ('pits filled', filled)):
        np.savez(tmp, elevation=-height.T, variance=np.full(h.shape, 1e-4),
                 count=np.full(h.shape, 100), resolution=t['resolution'],
                 origin_north=-half, origin_east=-half)
        rows = judge(tmp, pits, cells)
        print(f'{label}:')
        for p, depth in enumerate(t['plate_depths']):
            rs = [r for r in rows if r['plate'] == p]
            print(f'  plate depth {depth:.1f}: step_max '
                  f'{min(r["step_max_m"] for r in rs):.2f} to '
                  f'{max(r["step_max_m"] for r in rs):.2f} m, depth seen '
                  f'{np.median([r["depth_seen_m"] for r in rs]):.2f} m, '
                  f'landable {rs[0]["plate_landable"]}, confirmable '
                  f'{rs[0]["plate_confirm"]}')


def main():
    t, pits, cells = load_truth()
    depths = [float(d) for d in t['plate_depths']]
    if sys.argv[1:] == ['--truth']:
        truth_check(t, pits, cells)
        return
    rows = []
    for arg in sys.argv[1:]:
        rows += flight_rows(Path(arg), pits, cells, depths)
    out = Path(__file__).parent / 'pits.csv'
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    report(rows, depths)


if __name__ == '__main__':
    main()
