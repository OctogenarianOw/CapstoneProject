"""
B7, mechanism. Where is confirmed landing area gained, and how do the
C1 missions end?

Role: b7_mission_metric.py finds that the confirmed landing area rises
with alpha while coverage peaks inside the range. A zone is confirmed
on the second pass of a whole footprint, so the area should be gained
when the vehicle re-crosses ground it has seen. This script attributes
every increase of the confirmed landing area between two cycles to the
state of the vehicle in the later cycle (NAVIGATING to a frontier,
RETURNING, other), and counts per alpha how the missions ended: by an
abort of the current target (C1 monitor), by no admissible frontier at
selection (C1 filter), or otherwise.

Design choice: increases only (gross gain), since demotions are
analysed in Section 6.6; the share is computed over all B3 missions of
a world and alpha. This tests the explanation instead of asserting it.
"""
from collections import Counter

import numpy as np

from common import ALPHAS, CAMPAIGNS, load, missions_csv, runs
from b7_mission_metric import confirmed_landing_m2


def main():
    for world in CAMPAIGNS:
        rows = missions_csv(world)
        print(f'\n== {world}: gross confirmed landing area gained, by state')
        for cfg in ALPHAS:
            gain = Counter()
            ends = Counter()
            aborts = 0
            for c_, r, d in runs(world):
                if c_ != cfg:
                    continue
                rec = load(d)
                cyc = rec['cycles']
                prev = 0.0
                for c in cyc:
                    a = confirmed_landing_m2(c)
                    if a > prev:
                        st = c['state']
                        key = st if st in ('NAVIGATING', 'RETURNING') else 'other'
                        gain[key] += a - prev
                    prev = a
                ab = [e for e in rec['events'] if e['kind'] == 'abort']
                aborts += len(ab)
                why = rows[d.name]['end_why']
                ends[why] += 1
            tot = sum(gain.values())
            print(f'  {cfg}: total {tot:6.0f} m2; navigating '
                  f'{gain["NAVIGATING"] / tot * 100:4.0f} %, returning '
                  f'{gain["RETURNING"] / tot * 100:4.0f} %, other '
                  f'{gain["other"] / tot * 100:3.0f} % | aborts {aborts}, '
                  f'ends {dict(ends)}')


if __name__ == '__main__':
    main()
