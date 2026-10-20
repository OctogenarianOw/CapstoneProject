#!/usr/bin/env python3
"""
What a change of alpha during a mission did. Offline, from the logs.

Reads one session directory of run.sh: the 'alpha' events, every
frontier choice with the alpha in force, its G, M, the share of the
budget it leaves (m_share), its distance from the nearest confirmed
zone, its return energy and path distance, and the distance of the
aircraft to the nearest confirmed zone over time. Then every decision
re-ranked, from the candidates logged with it, at the starting alpha
and at the final one: a choice made after the change that differs from
the pick of the starting alpha is the change acting. Writes
alpha_live.png next to the session: the distance against time, the
choices marked, the change of alpha drawn.

    python3 alpha_live.py SESSION_DIR
"""

import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

run = Path(sys.argv[1])
mission = next((run / 'missions').iterdir())
config = json.loads((mission / 'config.json').read_text())
events = [json.loads(x) for x in (mission / 'events.jsonl').open()]
cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]

changes = [(e['t'], e['value']) for e in events if e['kind'] == 'alpha']
print(f'start alpha {config["alpha"]}, changes {changes}')


def alpha_at(t):
    a = config['alpha']
    for tc, v in changes:
        if tc <= t:
            a = v
    return a


def nearest_zone(cycle, ne):
    zones = [z['touchdown_ne'] for z in cycle['zones']
             if z['state'] == 'CONFIRMED']
    return min((math.hypot(ne[0] - a, ne[1] - b) for a, b in zones),
               default=math.nan)


times, dists, choices = [], [], []
for c in cycles:
    if c['state'] in ('INIT', 'TAKEOFF', 'LANDED'):
        continue
    times.append(c['t'])
    dists.append(nearest_zone(c, c['vehicle_ned']))
    chosen = [x for x in c.get('candidates') or [] if x.get('chosen')]
    if chosen and c['state'] == 'NAVIGATING' and (
            not choices or chosen[0]['viewpoint_ne'] != choices[-1][1]):
        x = chosen[0]
        pool = [y for y in c['candidates']
                if y.get('admissible') and y.get('g') is not None]
        choices.append((c['t'], x['viewpoint_ne'], x['g'], x['m'],
                        x.get('m_share', math.nan),
                        nearest_zone(c, x['viewpoint_ne']),
                        x['e_return'], x['path_dist_m'], pool))


def pick(pool, alpha):
    """Return the frontier C2 ranks first at this alpha, as evaluate()."""
    best = max(pool, key=lambda y: (round(alpha * y['g']
                                          + (1 - alpha) * y['m'], 12),
                                    -y['path_dist_m']))
    return tuple(best['viewpoint_ne'])


print(f'{"t s":>6} {"alpha":>5} {"frontier N, E":>15} {"G":>5} {"M":>5} '
      f'{"share":>6} {"to zone m":>9} {"E_return J":>10} {"path m":>6}')
for t, vp, g, m, share, dz, er, pd, _ in choices:
    print(f'{t:6.1f} {alpha_at(t):5.2f} {vp[0]:+7.1f},{vp[1]:+6.1f} '
          f'{g:5.2f} {m:5.2f} {share:6.3f} {dz:9.1f} {er:10.0f} {pd:6.1f}')

# The same decisions re-ranked at the starting alpha and at the final
# one, from the candidates logged with them: a choice after the change
# that differs from what the starting alpha picks is the change acting.
a_start = config['alpha']
a_end = changes[-1][1] if changes else a_start
print()
print(f'decision   in force   pick at alpha {a_start:g}   pick at alpha '
      f'{a_end:g}   choice changed by the switch')
for t, vp, *_, pool in choices:
    p0, p1 = pick(pool, a_start), pick(pool, a_end)
    after = alpha_at(t) != a_start
    print(f'{t:6.1f} s {alpha_at(t):9.2f}   {p0[0]:+6.1f},{p0[1]:+6.1f}'
          f'          {p1[0]:+6.1f},{p1[1]:+6.1f}          '
          f'{"yes" if after and p0 != p1 else "no" if after else "-"}')

fig, ax = plt.subplots(figsize=(9, 4))
ax.plot(times, dists, color='#1f4e79', lw=1.5,
        label='aircraft to nearest confirmed zone')
for t, vp, g, m, share, dz, er, pd, _ in choices:
    ax.plot(t, dz, 'o', color='#c0504d')
for tc, v in changes:
    ax.axvline(tc, color='#7f7f7f', ls='--')
    ax.text(tc, ax.get_ylim()[1] * 0.95, f' alpha -> {v:g}', va='top')
ax.set_xlabel('mission time, s')
ax.set_ylabel('distance, m')
ax.set_title(f'alpha {config["alpha"]:g} at start, budget '
             f'{cycles[0]["remaining_j"] / 1000:.0f} kJ; red: chosen '
             f'frontier, its distance to the nearest zone')
ax.legend(loc='upper left', fontsize=8)
fig.tight_layout()
fig.savefig(run / 'alpha_live.png', dpi=120)
print(f'figure {run / "alpha_live.png"}')
