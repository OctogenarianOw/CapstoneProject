#!/usr/bin/env python3
"""
P5 fix v5 on W1 surveys, non-inferiority on the flat RMS: the table and
the criterion of README.md, item by item.

The measures are those of 2026-10-21_p5_fix/analyse.py, whose functions
are reused so every P5 experiment is measured by the same code (ramp
alignment, flat RMS, recall and severe false accepts against the W1
truth, share of scans at the scan time), plus the clock steps logged
during each flight; a flight with a step does not count.

    python3 analyse.py        # ROS and the workspace sourced
"""

from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-10-21_p5_fix'))
from analyse import part_b, scans_at_stamp  # noqa: E402

DELTA_M = 0.02      # non-inferiority margin on the flat RMS (README.md)


def clock_steps(run):
    """Number of clock steps logged during the flight, -1 if no log."""
    log = run / 'clock.log'
    if not log.exists():
        return -1
    return sum(1 for line in log.read_text().splitlines()
               if line.startswith('step'))


def main():
    runs = sorted(p for p in (HERE / 'runs').iterdir()
                  if p.is_dir() and not p.name.startswith('.'))
    rows = {}
    print('B, W1 survey: ramps, flat, detector')
    print(f'{"run":<16} {"ramp RMS (0,0)":>14} {"best shift":>10} '
          f'{"flat RMS":>8} {"recall":>7} {"severe":>6} '
          f'{"at scan time":>13} {"clock steps":>11}')
    for run in runs:
        b = part_b(run / 'elevation_map.npz')
        b['at_stamp'] = scans_at_stamp(run)
        b['steps'] = clock_steps(run)
        rows[run.name] = b
        print(f'{run.name:<16} {b["ramp_rms_00"]:14.3f} '
              f'{str(b["best_shift"]):>10} {b["flat_rms"]:8.3f} '
              f'{b["recall"]:6.1f}% {b["severe"]:6d} '
              f'{b["at_stamp"]:12.1%} {b["steps"]:11d}')

    ok = {k: v for k, v in rows.items() if v['steps'] == 0}
    before = [v for k, v in ok.items() if '_before_' in k]
    after = [v for k, v in ok.items() if '_after_' in k]
    print(f'\ncounted (no clock step): {len(before)} before, {len(after)} '
          f'after')
    if not before or not after:
        return
    flat_b = statistics.median(v['flat_rms'] for v in before)
    flat_a = statistics.median(v['flat_rms'] for v in after)
    checks = [
        ('scans at the scan time over 95 % in every after flight',
         all(v['at_stamp'] > 0.95 for v in after)),
        ('best ramp shift (0, 0) in every after flight',
         all(v['best_shift'] == (0, 0) for v in after)),
        ('ramp RMS at (0, 0) of every after flight under the lowest before',
         max(v['ramp_rms_00'] for v in after)
         < min(v['ramp_rms_00'] for v in before)),
        ('recall of every after flight at least the lowest before - 2 pts',
         min(v['recall'] for v in after)
         >= min(v['recall'] for v in before) - 2.0),
        ('severe false accepts of every after flight at most the fewest '
         'before', max(v['severe'] for v in after)
         <= min(v['severe'] for v in before)),
        (f'flat RMS, median after {flat_a:.3f} m at most median before '
         f'{flat_b:.3f} m + {DELTA_M:.2f} m', flat_a <= flat_b + DELTA_M),
    ]
    print('\ncriterion (README.md)')
    for text, passed in checks:
        print(f'  {"met   " if passed else "FAILED"} {text}')
    print(f'\nall met: {all(p for _, p in checks)}')
    print(f'reported, not a condition: flat RMS under 0.25 m in '
          f'{sum(v["flat_rms"] < 0.25 for v in before)}/{len(before)} '
          f'before, {sum(v["flat_rms"] < 0.25 for v in after)}/'
          f'{len(after)} after')


if __name__ == '__main__':
    main()
