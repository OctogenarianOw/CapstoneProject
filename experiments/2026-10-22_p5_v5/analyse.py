#!/usr/bin/env python3
"""
P5 fix v3 on a stable clock: the measures of 2026-10-21_p5_fix, plus
the clock steps of each flight.

Same tables as ../2026-10-21_p5_fix/analyse.py, whose functions are
reused so both days are measured by the same code: plates (A, spill
and its direction-dependent part), W1 survey (B, ramp alignment, flat
RMS, recall, severe false accepts), share of scans at the scan time.
One column more, the steps of the WSL clock logged by clock_watch.py
during the flight; a flight with a step does not count (README.md).

    python3 analyse.py        # ROS and the workspace sourced
"""

from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / '2026-10-21_p5_fix'))
from analyse import part_b, scans_at_stamp, spills  # noqa: E402


def clock_steps(run):
    """Number of clock steps logged during the flight, -1 if no log."""
    log = run / 'clock.log'
    if not log.exists():
        return -1
    return sum(1 for line in log.read_text().splitlines()
               if line.startswith('step'))


def main():
    runs = sorted(p for p in list((HERE / 'runs').iterdir()) + list((HERE.parent / '2026-10-22_p5_v4' / 'runs').glob('*_before_*')) if p.is_dir()
                  and not p.name.startswith('.'))
    print('A, plates: spill (fraction of the plate height) and A')
    print(f'{"run":<14} {"after 4: N / S":>16} {"after 5: N / S":>16} '
          f'{"A":>7} {"at scan time":>13} {"clock steps":>11}')
    for run in [r for r in runs if r.name.startswith('A_')]:
        n4, s4 = spills(run / 'snap_line4.npz')
        n5, s5 = spills(run / 'snap_line5.npz')
        a = ((n5 - s5) - (n4 - s4)) / 2.0
        print(f'{run.name:<14} {n4:7.2f} / {s4:5.2f}  {n5:7.2f} / {s5:5.2f}'
              f' {a:+7.3f} {scans_at_stamp(run):12.1%} '
              f'{clock_steps(run):11d}')
    print('\nB, W1 survey: ramps, flat, detector')
    print(f'{"run":<14} {"ramp RMS (0,0)":>14} {"best shift":>10} '
          f'{"best RMS":>8} {"flat RMS":>8} {"recall":>7} {"severe":>6} '
          f'{"at scan time":>13} {"clock steps":>11}')
    for run in [r for r in runs if r.name.startswith('B_')]:
        b = part_b(run / 'elevation_map.npz')
        print(f'{run.name:<14} {b["ramp_rms_00"]:14.3f} '
              f'{str(b["best_shift"]):>10} {b["best_rms"]:8.3f} '
              f'{b["flat_rms"]:8.3f} {b["recall"]:6.1f}% {b["severe"]:6d} '
              f'{scans_at_stamp(run):12.1%} {clock_steps(run):11d}')


if __name__ == '__main__':
    main()
