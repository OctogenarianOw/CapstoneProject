"""
B3, B4, B6. What the filter and the utility actually do at each
decision, and what a decision costs to compute.

Role, from the per-cycle candidate records of the 210 final missions:
  B4  the fraction of the evaluated frontiers that C1 rejects at each
      decision (1 - |A| / |F|), the single number that says whether the
      filter bites, and how it evolves with the energy spent;
  B3  the distribution of |A| per decision, and whether C2 degenerates
      into a two-way switch at alpha = 0.5 (with |A| = 2 the per-cycle
      rescaling maps G and M to {0, 1}, so U is alpha or 1 - alpha and
      the choice is binary); also how often the alpha = 0.5 choice
      differs from both the pure-gain and the pure-margin choice,
      recomputed from the logged rescaled g and m;
  B6  the computation time of a cycle against the size of the problem
      (observed cells, frontiers evaluated), from timings_ms.

Design choice: a decision is a transition into NAVIGATING (the cycle
stamped just before the state event holds the candidates the decision
used), plus the cycles where no frontier was admissible and the vehicle
returned for budget. For B1 the admissible flag is still computed by
the policy code but not used, which tells how often C1 would have
intervened in B1's choices. Rejected: averaging over all cycles, which
weights long transits where the candidate list is only re-evaluated.
"""
import numpy as np
from scipy import stats

from common import ALPHAS, CAMPAIGNS, CONFIGS, fmt, load, runs


def decision_cycles(rec):
    cyc = rec['cycles']
    ts = np.array([c['t'] for c in cyc])
    out = []
    for e in rec['events']:
        if e['kind'] != 'state':
            continue
        nav = e['new'] == 'NAVIGATING'
        budget_ret = (e['new'] == 'RETURNING'
                      and ('admissible' in e.get('why', '')
                           or 'budget' in e.get('why', '')))
        if not (nav or budget_ret):
            continue
        before = np.nonzero(ts <= e['t'])[0]
        if before.size == 0:
            continue
        c = cyc[int(before[-1])]
        if e['t'] - c['t'] > 0.5 or not c['candidates']:
            continue
        out.append((c, nav))
    return out


def main():
    res = {}
    timing = []
    for world in CAMPAIGNS:
        for cfg, r, d in runs(world):
            rec = load(d)
            budget = rec['cycles'][0]['remaining_j']
            for c in rec['cycles']:
                if c['state'] in ('NAVIGATING', 'SELECTING', 'RETURNING',
                                  'BOOTSTRAP') and c['seen'] is not None:
                    timing.append((world, c['seen'], len(c['candidates']),
                                   c['compute_ms'],
                                   c['timings_ms'].get('field', 0.0),
                                   c['timings_ms'].get('decision', 0.0),
                                   c['timings_ms'].get('analysis', 0.0)))
            for c, nav in decision_cycles(rec):
                F = len(c['candidates'])
                A = [x for x in c['candidates'] if x['admissible']]
                ch = [x for x in c['candidates'] if x['chosen']]
                row = {'F': F, 'A': len(A), 'rej': 1 - len(A) / F,
                       'spent': 1 - c['remaining_j'] / budget,
                       'chosen_inadm': (None if not (nav and len(ch) == 1)
                                        else not ch[0]['admissible'])}
                if len(A) >= 2 and all(x['g'] is not None for x in A):
                    g = np.array([x['g'] for x in A])
                    m = np.array([x['m'] for x in A])
                    i_g, i_m = int(np.argmax(g)), int(np.argmax(m))
                    i_h = int(np.argmax(0.5 * g + 0.5 * m))
                    row['half_differs_both'] = i_h not in (i_g, i_m)
                    row['g_eq_m'] = i_g == i_m
                res.setdefault((world, cfg), []).append(row)

    print('B4 fraction of evaluated frontiers rejected by C1 at a decision; '
          'B3 |A| per decision')
    print(f'{"world":5s} {"config":7s} {"n":>5s} {"|F|":>16s} {"|A|":>16s} '
          f'{"rejected":>18s} {"rej>0":>6s} {"A=0":>5s} {"A=1":>5s} '
          f'{"A=2":>5s} {"A>=3":>5s}')
    for (world, cfg), rows in res.items():
        F = [x['F'] for x in rows]
        A = np.array([x['A'] for x in rows])
        rej = np.array([x['rej'] for x in rows])
        n = len(rows)
        print(f'{world:5s} {cfg:7s} {n:5d} {fmt(F, 0):>16s} {fmt(A, 0):>16s} '
              f'{fmt(rej):>18s} {np.mean(rej > 0) * 100:5.0f}% '
              f'{np.mean(A == 0) * 100:4.0f}% {np.mean(A == 1) * 100:4.0f}% '
              f'{np.mean(A == 2) * 100:4.0f}% {np.mean(A >= 3) * 100:4.0f}%')

    print('\nB4 frontier choices that C1 would have rejected (chosen and '
          'not admissible), per configuration')
    for (world, cfg), rows in res.items():
        flags = [x['chosen_inadm'] for x in rows
                 if x['chosen_inadm'] is not None]
        if flags:
            print(f'  {world} {cfg:7s}: {sum(flags):4d} of {len(flags):4d} '
                  f'choices ({np.mean(flags) * 100:.0f} %)')

    print('\nB4 rejection by share of the budget spent (B3, all alphas, '
          'all worlds): median rejected fraction')
    rows = [x for (w, c), v in res.items() if c in ALPHAS for x in v]
    spent = np.array([x['spent'] for x in rows])
    rej = np.array([x['rej'] for x in rows])
    for lo in np.arange(0, 1.0, 0.1):
        sel = (spent >= lo) & (spent < lo + 0.1)
        if sel.sum():
            print(f'  spent {lo:.1f}-{lo + 0.1:.1f}: n {sel.sum():4d}, '
                  f'median {np.median(rej[sel]):.2f}, mean '
                  f'{rej[sel].mean():.2f}, share with any rejection '
                  f'{np.mean(rej[sel] > 0) * 100:.0f} %')
    rho, p = stats.spearmanr(spent, rej)
    print(f'  Spearman spent vs rejected: rho {rho:+.2f} p {p:.2g}')

    print('\nB3 alpha = 0.5 recomputed on every B3 decision with |A| >= 2: '
          'share where it differs from both pure gain and pure margin')
    for world in CAMPAIGNS:
        rows = [x for (w, c), v in res.items() if w == world and c in ALPHAS
                for x in v if 'half_differs_both' in x]
        A = np.array([x['A'] for x in rows])
        hd = np.array([x['half_differs_both'] for x in rows])
        eq = np.array([x['g_eq_m'] for x in rows])
        print(f'  {world}: n {len(rows)}, |A| = 2 in {np.mean(A == 2) * 100:.0f}'
              f' %, gain and margin pick the same frontier in '
              f'{eq.mean() * 100:.0f} %, alpha 0.5 differs from both in '
              f'{hd.mean() * 100:.0f} % (|A| >= 3: '
              f'{hd[A >= 3].mean() * 100:.0f} %)')

    print('\nB6 computation time per cycle (ms) against observed cells')
    t = np.array([(x[1], x[2], x[3], x[4], x[5], x[6]) for x in timing], float)
    worlds = np.array([x[0] for x in timing])
    for world in CAMPAIGNS:
        s = t[worlds == world]
        print(f'  {world}: cycles {len(s)}, total {fmt(s[:, 2], 0)} ms, '
              f'p95 {np.percentile(s[:, 2], 95):.0f}, max {s[:, 2].max():.0f};'
              f' field {fmt(s[:, 3], 0)}, decision {fmt(s[:, 4], 0)}, '
              f'analysis {fmt(s[:, 5], 0)}')
    for lo, hi in ((0, 2000), (2000, 5000), (5000, 10000), (10000, 15000),
                   (15000, 20000), (20000, 30000)):
        sel = (t[:, 0] >= lo) & (t[:, 0] < hi)
        if sel.sum():
            print(f'  seen {lo:5d}-{hi:5d}: n {sel.sum():5d}, total median '
                  f'{np.median(t[sel, 2]):6.0f} ms, p95 '
                  f'{np.percentile(t[sel, 2], 95):6.0f}, max '
                  f'{t[sel, 2].max():6.0f}')
    rho, p = stats.spearmanr(t[:, 0], t[:, 2])
    print(f'  Spearman seen cells vs total time: rho {rho:+.2f}')
    rho, p = stats.spearmanr(t[:, 1], t[:, 2])
    print(f'  Spearman candidates vs total time: rho {rho:+.2f}')
    # least-squares slope of total time on candidates, W3 only
    s = t[worlds == 'W3']
    k, b = np.polyfit(s[:, 1], s[:, 2], 1)
    print(f'  W3 linear fit total = {b:.0f} + {k:.1f} ms per candidate')
    np.savez('b6_timing.npz', seen=t[:, 0], cand=t[:, 1], total=t[:, 2],
             field=t[:, 3], decision=t[:, 4], analysis=t[:, 5], world=worlds)
    rows = [(w, c, x['spent'], x['rej'], x['A'], x['F'])
            for (w, c), v in res.items() for x in v]
    np.savez('b4_rejection.npz', world=[r[0] for r in rows],
             config=[r[1] for r in rows], spent=[r[2] for r in rows],
             rej=[r[3] for r in rows], A=[r[4] for r in rows],
             F=[r[5] for r in rows])


if __name__ == '__main__':
    main()
