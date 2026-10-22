#!/usr/bin/env python3
"""Read, and with --fix reset, the kernel clock tick of WSL (trap 22).

On 22 Oct 2026, after the host woke from standby, the WSL system clock
ran 2.56 % fast and Hyper-V stepped it back by about 0.8 s every 32 s,
which spoils every duration measured on the system clock (decision
latency, energy integration, PX4 stamps against simulation time). The
cause, read with adjtimex: the kernel tick was 10256 us instead of the
nominal 10000 (10000 / USER_HZ 100), so each tick added 2.56 % too much
time. Neither the clock source nor a `wsl --shutdown` changed it.
--fix sets the tick back to 10000 (root needed); the frequency
correction, a few tens of ppm, is the time service's and is left alone.

    python3 tools/wsl_clock_tick.py            # read
    sudo python3 tools/wsl_clock_tick.py --fix # reset the tick
"""

import ctypes
import ctypes.util
import sys

ADJ_TICK = 0x4000
NOMINAL_TICK_US = 10000


class Timex(ctypes.Structure):
    """struct timex of <sys/timex.h>, x86_64."""

    _fields_ = [('modes', ctypes.c_uint), ('offset', ctypes.c_long),
                ('freq', ctypes.c_long), ('maxerror', ctypes.c_long),
                ('esterror', ctypes.c_long), ('status', ctypes.c_int),
                ('constant', ctypes.c_long), ('precision', ctypes.c_long),
                ('tolerance', ctypes.c_long), ('time_sec', ctypes.c_long),
                ('time_usec', ctypes.c_long), ('tick', ctypes.c_long),
                ('ppsfreq', ctypes.c_long), ('jitter', ctypes.c_long),
                ('shift', ctypes.c_int), ('stabil', ctypes.c_long),
                ('jitcnt', ctypes.c_long), ('calcnt', ctypes.c_long),
                ('errcnt', ctypes.c_long), ('stbcnt', ctypes.c_long),
                ('tai', ctypes.c_int), ('pad', ctypes.c_int * 11)]


def adjtimex(tmx):
    """Call adjtimex, raise on error."""
    libc = ctypes.CDLL(ctypes.util.find_library('c'), use_errno=True)
    if libc.adjtimex(ctypes.byref(tmx)) < 0:
        raise OSError(ctypes.get_errno(), 'adjtimex failed (root needed?)')
    return tmx


def report(tmx):
    """One line: tick, the rate error it makes, frequency correction."""
    rate = 100.0 * (tmx.tick - NOMINAL_TICK_US) / NOMINAL_TICK_US
    print(f'tick {tmx.tick} us (nominal {NOMINAL_TICK_US}, clock rate '
          f'{rate:+.2f} %), freq {tmx.freq / 65536.0:+.1f} ppm')


def main():
    tmx = adjtimex(Timex())
    report(tmx)
    if '--fix' in sys.argv[1:]:
        fix = Timex()
        fix.modes = ADJ_TICK
        fix.tick = NOMINAL_TICK_US
        adjtimex(fix)
        report(adjtimex(Timex()))


if __name__ == '__main__':
    main()
