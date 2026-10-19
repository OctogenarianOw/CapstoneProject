#!/usr/bin/env python3
"""Count PX4 failsafe activations for one recorded offboard session.

Two independent counts, because neither is complete on its own.

px4.log: lines "Failsafe activated", the count that gave 177 on 17 Oct.
When marks.txt exists (session.sh writes it), the count is split into
before the node started, while it was publishing, and after it was
stopped. The split matters: stopping the node ends the setpoint stream,
and PX4 is then right to raise one failsafe. Counting the whole log
would charge that one to the node.

state.csv: exits from nav_state 14 (offboard) while armed, as sampled
by the probe from vehicle_status. A flip shorter than the sampling is
missed, so this count is a lower bound and only corroborates the log.

    python3 count_failsafes.py RUN_DIR
"""

import sys
from pathlib import Path

import numpy as np

NAV_OFFBOARD = 14
ARMING_ARMED = 2


def log_counts(run):
    lines = (run / 'px4.log').read_text(errors='replace').splitlines()
    hits = [i for i, line in enumerate(lines) if 'Failsafe activated' in line]
    print(f'px4.log failsafe activations       {len(hits)}')

    marks_path = run / 'marks.txt'
    if not marks_path.exists():
        print('  no marks.txt, no split by node lifetime')
        return
    marks = {}
    for row in marks_path.read_text().splitlines():
        if row.strip():
            name, wall, n_lines = row.split()
            marks[name] = (float(wall), int(n_lines))
    start = marks['node_start'][1]
    stop = marks['node_stop'][1]
    before = sum(1 for i in hits if i < start)
    during = sum(1 for i in hits if start <= i < stop)
    after = len(hits) - before - during
    wall = marks['node_stop'][0] - marks['node_start'][0]
    print(f'  before node start                {before}')
    print(f'  while node publishing            {during}   ({wall:.1f} s wall)')
    print(f'  after node stop                  {after}')
    for i in hits:
        if i >= stop:
            context = ' | '.join(s.strip() for s in lines[max(0, i - 2):i + 2])
            print(f'    after stop, line {i + 1}: {context}')


def state_counts(run):
    s = np.genfromtxt(run / 'state.csv', delimiter=',', names=True)
    t = s['t_px4']
    armed = s['arming'] == ARMING_ARMED
    if not armed.any():
        print('state.csv: never armed')
        return
    t_arm = t[np.argmax(armed)]
    print(f'state.csv armed span               {t[armed][-1] - t_arm:.1f} s')
    nav = s['nav']
    exits = np.flatnonzero(armed[1:] & (nav[:-1] == NAV_OFFBOARD)
                           & (nav[1:] != NAV_OFFBOARD)) + 1
    print(f'  offboard exits while armed       {len(exits)}   (lower bound)')
    if len(exits):
        rel = t[exits] - t_arm
        print(f'  first at {rel[0]:.1f} s, last at {rel[-1]:.1f} s after arming')
        print(f'  nav_state after last exit        {int(nav[exits[-1]])}')
    in_offboard = armed & (nav == NAV_OFFBOARD)
    print(f'  armed samples in offboard        '
          f'{100.0 * in_offboard.sum() / armed.sum():.1f} %')


def main():
    run = Path(sys.argv[1])
    print(f'== {run.name}')
    log_counts(run)
    state_counts(run)


if __name__ == '__main__':
    main()
