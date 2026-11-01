"""Build the compact cache of the 210 final missions and the 13 S9.5 runs.

Run once; every analysis script then reads cache/. Prints the count and
the parse time so that a missing or truncated log is seen at once.
"""
import time
from pathlib import Path

from common import CAMPAIGNS, S95, load, runs

t0 = time.time()
n = 0
for world in CAMPAIGNS:
    for cfg, r, d in runs(world):
        rec = load(d)
        n += 1
        if not rec['cycles']:
            print('EMPTY', world, d.name)
    print(world, 'done', n, f'{time.time() - t0:.0f} s', flush=True)
for d in sorted((S95 / 'runs').iterdir()):
    if (d / 'missions').is_dir():
        load(d)
        n += 1
print('total', n, f'{time.time() - t0:.0f} s')
