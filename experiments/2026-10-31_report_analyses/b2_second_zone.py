"""
B2. How often would a stricter guarantee, one that does not depend on
the zone in use, have held?

Role: Section 6.6 shows that I1 holds as long as the registry does not
shrink under the zone the vehicle relies on. The natural objection is
that a guarantee resting on one zone is weaker than it looks. This
script measures, over every frontier decision of the 150 C1 missions,
whether the chosen target would still have been admissible through the
second-best confirmed zone (need_without <= budget), using the
'guarantee' events that the exploration manager logged in flight for
every decision whose return zone was a discovered one (mission_fsm.py,
_log_guarantee). Decisions whose return zone was the provisional
take-off point are counted separately: no event is logged for them,
and the stricter guarantee would then need any discovered zone at all.

Design choice: count decisions, not cycles. The logged quantity is
evaluated at each decision, from the viewpoint the decision targets;
between decisions the monitor checks I1 but does not log the
second-zone margin. Rejected: recomputing a second-zone return field
offline for every cycle from the 10 s map snapshots, which would not
be the map the decision saw.
"""
import numpy as np

from common import ALPHAS, CAMPAIGNS, load, runs


def decisions(rec):
    """(time, kind, event) for every transition into NAVIGATING."""
    ev = rec['events']
    g = [e for e in ev if e['kind'] == 'guarantee']
    navs = [e for e in ev if e['kind'] == 'state' and e['new'] == 'NAVIGATING']
    out = []
    for n in navs:
        # the guarantee event of a decision precedes its state event
        # within the same call (same inp.t)
        match = [x for x in g if abs(x['t'] - n['t']) < 1e-6]
        if match:
            x = match[0]
            out.append(('discovered', x))
        else:
            out.append(('takeoff', None))
    return out


def main():
    print('decisions of the C1 missions (B3, all alphas): return zone, and '
          'whether the target stays admissible without that zone')
    tot = {}
    for world in CAMPAIGNS:
        per = {}
        for cfg, r, d in runs(world):
            if cfg not in ALPHAS:
                continue
            rec = load(d)
            ds = decisions(rec)
            s = per.setdefault(cfg, {'n': 0, 'takeoff': 0, 'strict': 0,
                                     'rests': 0, 'no_other': 0,
                                     'i1_without': [], 'missions_all_strict': 0,
                                     'missions': 0})
            s['missions'] += 1
            all_strict = True
            for kind, x in ds:
                s['n'] += 1
                if kind == 'takeoff':
                    s['takeoff'] += 1
                    all_strict = False
                    continue
                if not np.isfinite(x['need_without_j']):
                    s['no_other'] += 1
                    s['rests'] += 1
                    all_strict = False
                elif x['rests_on_zone']:
                    s['rests'] += 1
                    all_strict = False
                else:
                    s['strict'] += 1
                if np.isfinite(x['i1_without_j']):
                    s['i1_without'].append(x['i1_without_j'])
            s['missions_all_strict'] += all_strict
        print(f'\n== {world}')
        print('config   decisions  take-off zone  discovered: strict holds / '
              'rests on zone (no other zone)   I1 without zone at decision, '
              'median kJ, share < 0')
        for cfg, s in per.items():
            disc = s['n'] - s['takeoff']
            iw = np.array(s['i1_without']) / 1000
            print(f'{cfg:8s} {s["n"]:9d}  {s["takeoff"]:13d}  '
                  f'{s["strict"]:5d} / {s["rests"]:4d} ({s["no_other"]:3d})'
                  f'   strict share of discovered '
                  f'{(s["strict"] / disc * 100) if disc else float("nan"):5.1f} %'
                  f'   {np.median(iw) if iw.size else float("nan"):6.2f}, '
                  f'{(np.mean(iw < 0) * 100) if iw.size else float("nan"):4.0f} %')
            for k in ('n', 'takeoff', 'strict', 'rests', 'no_other'):
                tot.setdefault(world, {}).setdefault(k, 0)
                tot[world][k] += s[k]
        t = tot[world]
        print(f'  {world} total: {t["n"]} decisions, {t["takeoff"]} on the '
              f'take-off zone, {t["strict"]} strict, {t["rests"]} resting on '
              f'one zone ({t["no_other"]} with no other zone confirmed)')
    n = sum(t['n'] for t in tot.values())
    st = sum(t['strict'] for t in tot.values())
    tk = sum(t['takeoff'] for t in tot.values())
    rs = sum(t['rests'] for t in tot.values())
    no = sum(t['no_other'] for t in tot.values())
    print(f'\nall worlds: {n} decisions; take-off zone {tk} '
          f'({tk / n * 100:.0f} %); discovered zone {n - tk}: strict holds '
          f'{st} ({st / (n - tk) * 100:.0f} %), rests on its zone {rs} '
          f'({rs / (n - tk) * 100:.0f} %), of which {no} with no other '
          f'confirmed zone')


if __name__ == '__main__':
    main()
