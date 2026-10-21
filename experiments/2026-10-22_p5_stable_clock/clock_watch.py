#!/usr/bin/env python3
"""
Log every step of the WSL system clock while a flight runs (trap 22).

CLOCK_REALTIME minus CLOCK_MONOTONIC is constant on a sane clock; on
21 Oct it jumped by about -0.7 s every 32 s. Sampled every 0.1 s, any
change over 0.05 s is printed with its time, so each run directory
says whether its flight was flown on a stable clock. Run with a
duration it is the pre-flight check: exit 1 if the clock stepped.

    python3 clock_watch.py            # until killed, one line per step
    python3 clock_watch.py 60         # check for 60 s, exit 1 on a step
"""

import sys
import time

THRESHOLD_S = 0.05


def offset():
    """System clock minus the monotonic clock, in seconds."""
    return (time.clock_gettime(time.CLOCK_REALTIME)
            - time.clock_gettime(time.CLOCK_MONOTONIC))


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else float('inf')
    t0 = time.monotonic()
    prev = offset()
    steps = 0
    print(f'clock watch start {time.strftime("%H:%M:%S")}', flush=True)
    while time.monotonic() - t0 < duration:
        time.sleep(0.1)
        now = offset()
        if abs(now - prev) > THRESHOLD_S:
            steps += 1
            print(f'step {time.strftime("%H:%M:%S")} '
                  f'{now - prev:+.3f} s at {time.monotonic() - t0:.1f} s',
                  flush=True)
        prev = now
    print(f'clock watch end, {steps} steps', flush=True)
    sys.exit(1 if steps else 0)


if __name__ == '__main__':
    main()
