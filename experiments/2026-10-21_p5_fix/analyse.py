#!/usr/bin/env python3
"""
P5 fix, before and after: the plates (A) and the ramps of W1 (B).

A, world pits (runs A_before_rN, A_after_rN): north and south spill of
the plate edges after lines 4 (flown south) and 5 (flown north), as in
2026-10-21_p5_latency, and the direction-dependent part
    A = ((north - south spill after line 5)
         - (north - south spill after line 4)) / 2
with the share of scans that cloud_world fused with the pose at the
scan time (last count of map.log).

B, world validation (runs B_before_rN, B_after_rN): the tools that
measured P5 and the safety of the detector on the reference map v6,
on each final map: check_alignment.py (ramp RMS at shift (0, 0), best
shift and its RMS), validate_map.py (flat RMS, criterion 0.25 m of S5),
validate_slz.py (recall, severe false accepts, barrier of S8).

Criterion, fixed before the flights (README.md): the fix works if |A|
falls under 0.1 in every flight after (about +0.45 before) with more
than 95 % of the scans at the scan time, and if on W1 the best ramp
shift is (0, 0) after, with a lower ramp RMS at (0, 0) than before; the
flat RMS and the severe false accepts must not get worse.

    python3 analyse.py
"""

from pathlib import Path
import re
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
TOOLS = HERE.parents[1] / 'tools'
PLATES = [(-36.0, 0.5), (-12.0, 1.0), (12.0, 1.5), (36.0, 2.0)]
HALF = 6.0
COLS = (36, 44)
GROUND = (-4, 4, 48, 56)


def spills(path):
    """North and south spill, mean over the plates, of one map."""
    m = np.load(path)
    up = -m['elevation'].astype(float)
    ox, oy = float(m['origin_x']), float(m['origin_y'])
    g = up[int(GROUND[0] - ox):int(GROUND[1] - ox),
           int(GROUND[2] - oy):int(GROUND[3] - oy)]
    ref = np.nanmedian(g)
    cols = slice(int(COLS[0] - oy), int(COLS[1] - oy))
    n = [np.nanmean(up[int(c + HALF - ox), cols] - ref) / d for c, d in PLATES]
    s = [np.nanmean(up[int(c - HALF - 1 - ox), cols] - ref) / d
         for c, d in PLATES]
    return float(np.mean(n)), float(np.mean(s))


def scans_at_stamp(run):
    """Share of scans fused with the pose at the scan time."""
    hits = re.findall(r'pose at stamp (\d+), latest (\d+)',
                      (run / 'map.log').read_text(errors='replace'))
    if not hits:
        return float('nan')
    a, b = map(int, hits[-1])
    return a / max(1, a + b)


def tool(name, path):
    """Output of one tool of tools/ on one map."""
    return subprocess.run([sys.executable, str(TOOLS / name), str(path)],
                          capture_output=True, text=True).stdout


def part_b(path):
    """Ramp alignment, flat RMS, recall and severe false accepts."""
    out = tool('check_alignment.py', path).split('--- all terrain')[0]
    rows = {}
    for line in out.splitlines():
        m = re.match(r'di ([+-]\d) (.*)', line)
        if m:
            rows[int(m.group(1))] = [float(x) for x in m.group(2).split()]
    best = re.search(r'best: RMS ([\d.]+) at shift \((-?\d+), (-?\d+)\)',
                     out)
    flat = re.search(r'^flat\s+\d+\s+\S+\s+\S+\s+([\d.]+)',
                     tool('validate_map.py', path), re.M)
    slz = tool('validate_slz.py', path)
    recall = re.search(r'recall\s+([\d.]+)%', slz)
    severe = re.search(r'(\d+) severe', slz)
    return {'ramp_rms_00': rows[0][3],
            'best_rms': float(best.group(1)),
            'best_shift': (int(best.group(2)), int(best.group(3))),
            'flat_rms': float(flat.group(1)),
            'recall': float(recall.group(1)),
            'severe': int(severe.group(1))}


def main():
    runs = sorted(p for p in (HERE / 'runs').iterdir() if p.is_dir()
                  and not p.name.startswith('.'))
    print('A, plates: spill (fraction of the plate height) and A')
    print(f'{"run":<16} {"after 4: N / S":>16} {"after 5: N / S":>16} '
          f'{"A":>7} {"at scan time":>13}')
    for run in [r for r in runs if r.name.startswith('A_')]:
        n4, s4 = spills(run / 'snap_line4.npz')
        n5, s5 = spills(run / 'snap_line5.npz')
        a = ((n5 - s5) - (n4 - s4)) / 2.0
        print(f'{run.name:<16} {n4:7.2f} / {s4:5.2f}  {n5:7.2f} / {s5:5.2f}'
              f' {a:+7.3f} {scans_at_stamp(run):12.1%}')
    print('\nB, W1 survey: ramps, flat, detector')
    print(f'{"run":<16} {"ramp RMS (0,0)":>14} {"best shift":>10} '
          f'{"best RMS":>8} {"flat RMS":>8} {"recall":>7} {"severe":>6} '
          f'{"at scan time":>13}')
    for run in [r for r in runs if r.name.startswith('B_')]:
        b = part_b(run / 'elevation_map.npz')
        print(f'{run.name:<16} {b["ramp_rms_00"]:14.3f} '
              f'{str(b["best_shift"]):>10} {b["best_rms"]:8.3f} '
              f'{b["flat_rms"]:8.3f} {b["recall"]:6.1f}% {b["severe"]:6d} '
              f'{scans_at_stamp(run):12.1%}')
    v6 = part_b(HERE.parents[1] / 'data_survey_v6.npz')
    print(f'{"v6 (reference)":<16} {v6["ramp_rms_00"]:14.3f} '
          f'{str(v6["best_shift"]):>10} {v6["best_rms"]:8.3f} '
          f'{v6["flat_rms"]:8.3f} {v6["recall"]:6.1f}% {v6["severe"]:6d}')


if __name__ == '__main__':
    main()
