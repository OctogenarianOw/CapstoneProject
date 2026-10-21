#!/usr/bin/env python3
"""
Second pit calibration: do wider pits hide from afar and show in hover?

For every flight (pits2_drive.py on worlds/pits2.sdf) and every pit of
worlds/pits2_truth.npz, what the map makes of the pit after each line
and after the hover over its plate, against how close the aircraft had
come to it. Readings, as in 2026-10-21_pit_calibration/analyse_pits.py:

  step_max    largest step layer (S7) over the pit cells grown by one
              cell (one cell of P5 offset tolerated)
  depth_seen  median plate top minus the lowest observed cell there
  confirm     the plate holds a whole footprint element of landable
              cells on OBSERVED cells (the min-zone rule of the registry)
  dist        closest horizontal approach so far to the nearest point of
              the pit, not its centre: pits are now 2 and 3 m wide

Criterion, fixed on 21 Oct before these flights. A geometry (pit width,
depth) works for W3 if, in every flight:
  hidden    every line reading with the pit passed at 10 m or more has
            step_max < 0.24 m on an observed block: the classifier takes
            it as flat while the registry counts its passes (a pass at
            14.4 m from the centre of a 16 m pad comes within about 10 m
            of its nearest pits)
  revealed  after the hover over its plate, every pit has step_max
            > 0.48 m (past the severe limit of S8) and the plate holds no
            whole footprint: the zone would be withdrawn
Among the geometries that work, W3 takes the one hidden down to the
smallest distance, then the narrower pit, then the shallower. If none
works, W3 is not built on pits, and that result is reported as it is.
The pass within 5 m of the first calibration is printed for comparison,
not used: a pass at 5 m/s is not the approach to a landing.

    python3 analyse_pits2.py RUN_DIR [RUN_DIR ...]
    python3 analyse_pits2.py --truth    exact and filled truth, same chain
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

TRUTH = ROOT / 'worlds' / 'pits2_truth.npz'
HIDDEN_FROM_M = 10.0
HIDDEN_STEP_M = 0.24
REVEALED_STEP_M = 0.48


def load_truth():
    """Pits (cells, rectangle) and plate cells, NED, per plate."""
    t = np.load(TRUTH)
    c = t['centres']
    names = [str(s) for s in t['zone_names']]
    zone, plate, pit = t['zone'], t['plate'], t['pit']
    cells = {}
    for ix, iy in zip(*np.nonzero(plate >= 0)):
        cells.setdefault(int(plate[ix, iy]), []).append(
            ((float(c[iy]), float(c[ix])), names[zone[ix, iy]]))
    pits = []
    for k in np.unique(pit[pit >= 0]):
        ix, iy = np.nonzero(pit == k)
        ne = [(float(c[b]), float(c[a])) for a, b in zip(ix, iy)]
        n, e = np.array(ne).T
        pits.append({'pit': int(k), 'plate': int(plate[ix[0], iy[0]]),
                     'cells': ne, 'box': (n.min() - 0.5, n.max() + 0.5,
                                          e.min() - 0.5, e.max() + 0.5)})
    return t, pits, cells


def index(ne, origin, res):
    """Map cell (i north, j east) holding a NED point."""
    return (int(math.floor((ne[0] - origin[0]) / res)),
            int(math.floor((ne[1] - origin[1]) / res)))


def judge(snapshot, pits, cells):
    """Per pit readings of one map snapshot, and plate confirmability."""
    m = np.load(snapshot)
    keys = (('origin_north', 'origin_east') if 'origin_north' in m.files
            else ('origin_x', 'origin_y'))
    origin = (float(m[keys[0]]), float(m[keys[1]]))
    elev, var, count = m['elevation'], m['variance'], m['count']
    res = float(m['resolution'])
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
        plates[p] = {'top': float(np.median(tops)) if tops else math.nan,
                     'confirm': bool(any(core[i, j] for i, j in idx))}
    rows = []
    for q in pits:
        grown = np.zeros(elev.shape, dtype=bool)
        for ne in q['cells']:
            i, j = index(ne, origin, res)
            grown[i - 1:i + 2, j - 1:j + 2] = True
        steps = layers['step'][grown]
        seen = grown & (count > 0)
        rows.append({
            'pit': q['pit'], 'plate': q['plate'],
            'step_max_m': (float(np.nanmax(steps))
                           if np.isfinite(steps).any() else math.nan),
            'depth_seen_m': (plates[q['plate']]['top'] - float(up[seen].min())
                             if seen.any() else math.nan),
            'block_unobserved': bool((state[grown] != OBSERVED).any()),
            'plate_confirm': plates[q['plate']]['confirm'],
        })
    return rows


def read_csv(path):
    """Rows of a csv file as dicts."""
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def box_distance(n, e, box):
    """Horizontal distance from points to a rectangle (0 inside)."""
    n0, n1, e0, e1 = box
    dn = np.maximum(np.maximum(n0 - n, 0.0), n - n1)
    de = np.maximum(np.maximum(e0 - e, 0.0), e - e1)
    return np.hypot(dn, de)


def flight_rows(run, pits, cells, t):
    """Every pit at every snapshot of one flight, with its distance."""
    track = np.array([[float(r['t']), float(r['north']), float(r['east'])]
                      for r in read_csv(run / 'track.csv')])
    snaps = [(r['phase'][5:], float(r['t']))
             for r in read_csv(run / 'phases.csv')
             if r['phase'].startswith('snap_')]
    boxes = {q['pit']: q['box'] for q in pits}
    out = []
    for label, t_snap in snaps:
        upto = track[track[:, 0] <= t_snap]
        for row in judge(run / f'snap_{label}.npz', pits, cells):
            if label.startswith('hover') and int(label[5:]) != row['plate']:
                continue
            d = box_distance(upto[:, 1], upto[:, 2], boxes[row['pit']])
            p = row['plate']
            row.update(run=run.name, snap=label,
                       width_m=int(t['plate_widths'][p]),
                       depth_m=float(t['plate_depths'][p]),
                       min_dist_m=float(d.min()))
            out.append(row)
    return out


def hidden(r):
    """The classifier would take the pit as flat."""
    return (not r['block_unobserved'] and np.isfinite(r['step_max_m'])
            and r['step_max_m'] < HIDDEN_STEP_M)


def revealed(r):
    """The pit is refused past the severe limit."""
    return np.isfinite(r['step_max_m']) and r['step_max_m'] > REVEALED_STEP_M


def hidden_down_to(rows):
    """Smallest distance from which every line reading is hidden."""
    out = math.inf
    for r in sorted(rows, key=lambda r: -r['min_dist_m']):
        if not hidden(r):
            break
        out = r['min_dist_m']
    return out


def report(rows, t):
    """Criterion per geometry, the medians behind it, the choice."""
    geoms = [(int(w), float(d)) for w, d in zip(t['plate_widths'],
                                                t['plate_depths'])]
    print(f'{"width":>5} {"depth":>5} {"far reads":>9} {"hidden":>6} '
          f'{"hover reads":>11} {"revealed":>8} {"plate kept":>10}  '
          f'works  hidden down to')
    works = []
    for w, d in geoms:
        rs = [r for r in rows if r['width_m'] == w and r['depth_m'] == d]
        lines = [r for r in rs if r['snap'].startswith('line')]
        far = [r for r in lines if r['min_dist_m'] >= HIDDEN_FROM_M]
        hov = [r for r in rs if r['snap'].startswith('hover')]
        nh, nr = sum(map(hidden, far)), sum(map(revealed, hov))
        kept = sum(r['plate_confirm'] for r in hov)
        ok = bool(far and hov and nh == len(far) and nr == len(hov)
                  and kept == 0)
        down = hidden_down_to(lines)
        if ok:
            works.append((down, w, d))
        print(f'{w:5d} {d:5.1f} {len(far):9d} {nh:6d} {len(hov):11d} '
              f'{nr:8d} {kept:10d}  {"yes" if ok else "no":5s}  '
              f'{down:5.1f} m')
    print('\nstep_max / depth seen, median over pits and flights, by '
          'distance to the pit edge (line readings), then in hover')
    bins = [(0, 2), (2, 5), (5, 8), (8, 10), (10, 13), (13, 17), (17, 99)]
    print(f'{"geom":>9} ' + ' '.join(f'{f"{a}-{b} m":>15}' for a, b in bins)
          + f' {"hover":>15}')
    for w, d in geoms:
        cells = []
        rs = [r for r in rows if r['width_m'] == w and r['depth_m'] == d]
        groups = [[r for r in rs if r['snap'].startswith('line')
                   and a <= r['min_dist_m'] < b] for a, b in bins]
        groups.append([r for r in rs if r['snap'].startswith('hover')])
        for g in groups:
            s = [r['step_max_m'] for r in g if np.isfinite(r['step_max_m'])]
            ds = [r['depth_seen_m'] for r in g
                  if np.isfinite(r['depth_seen_m'])]
            cells.append(f'{np.median(s):.2f}/{np.median(ds):.2f} n{len(g)}'
                         if s and ds else f'n{len(g)}')
        print(f'{f"{w}m {d:.1f}m":>9} ' + ' '.join(f'{c:>15}' for c in cells))
    print('\nplate confirmable after each line, then after its hover')
    for run in sorted({r['run'] for r in rows}):
        for p, (w, d) in enumerate(geoms):
            seq = []
            for snap in ['line1', 'line2', 'line3', 'line4', 'line5',
                         f'hover{p}']:
                rs = [r for r in rows if r['run'] == run
                      and r['snap'] == snap and r['plate'] == p]
                seq.append('C' if rs and rs[0]['plate_confirm'] else '.')
            print(f'{run:<6} {w} m {d:.1f} m  {"".join(seq[:5])} | {seq[5]}')
    if works:
        down, w, d = min(works)
        print(f'\nW3 geometry: pits {w} m wide, {d:.1f} m deep, hidden '
              f'down to {down:.1f} m')
    else:
        print('\nW3 geometry: none meets the criterion')


def truth_check(t, pits, cells):
    """The exact truth and the filled truth through the same chain."""
    tmp = Path('/tmp/pits2_truth_as_map.npz')
    h = t['truth']
    depth = np.array(t['plate_depths'])[np.maximum(t['plate'], 0)]
    filled = np.where(t['zone'] == 2, depth, h)
    half = len(t['centres']) * float(t['resolution']) / 2.0
    for label, height in (('exact truth', h), ('pits filled', filled)):
        np.savez(tmp, elevation=-height.T, variance=np.full(h.shape, 1e-4),
                 count=np.full(h.shape, 100), resolution=t['resolution'],
                 origin_north=-half, origin_east=-half)
        rows = judge(tmp, pits, cells)
        print(f'{label}:')
        for p in range(len(t['plate_depths'])):
            rs = [r for r in rows if r['plate'] == p]
            print(f'  plate {p} ({t["plate_widths"][p]} m, '
                  f'{t["plate_depths"][p]:.1f} m): step_max '
                  f'{min(r["step_max_m"] for r in rs):.2f} to '
                  f'{max(r["step_max_m"] for r in rs):.2f} m, depth seen '
                  f'{np.median([r["depth_seen_m"] for r in rs]):.2f} m, '
                  f'confirmable {rs[0]["plate_confirm"]}')


def main():
    """Read the flights given, or check the truth."""
    t, pits, cells = load_truth()
    if sys.argv[1:] == ['--truth']:
        truth_check(t, pits, cells)
        return
    rows = []
    for arg in sys.argv[1:]:
        rows += flight_rows(Path(arg), pits, cells, t)
    with open(Path(__file__).parent / 'pits2.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    report(rows, t)


if __name__ == '__main__':
    main()
