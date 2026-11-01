"""
B1. Predicted against measured energy of the legs C1 reasons about, in
flight, over the 210 missions of the final campaigns.

Role: the zero violations of C1 rest on predicted energies. Section 4.5
validates the model on calibration flights; this script checks the
predictions the planner actually made in missions:
  - E_ret: the path energy logged by the last 'return' event of each
    mission (the leg that ended in the landing commit), against the
    energy measured from that event to the COMMIT_LANDING transition,
    with and without the latency charge tau * P_hover that C1 adds;
  - E_land: the planning constant 3387 J against the energy measured
    from COMMIT_LANDING to LANDED;
  - E_go: for every frontier chosen, the logged e_go (path plus tau
    hover) against the energy measured from the decision to the
    'arrived' transition.

Design choice: the energy at an event time is interpolated linearly
between the two 1 Hz cycles around it (consumed_j is the integral of
rotor shaft power, monotone). The error this adds is bounded by the
power variation within one second, a few tens of joules. Only the last
return leg is used: earlier legs end in a switch or a re-plan, so their
measured energy belongs to a path that was not flown to its end.
Transits that ended by timeout, blacklist or abort are excluded and
counted. Rejected: re-simulating the paths offline, which would test
the model and not what the planner used.
"""
import numpy as np

from common import CAMPAIGNS, fmt, load, runs

E_LAND_PLAN = 3387.0
P_HOVER = 285.636


def consumed_at(cycles, t):
    ts = np.array([c['t'] for c in cycles])
    e = np.array([c['consumed_j'] for c in cycles])
    return float(np.interp(t, ts, e))


def state_changes(events):
    return [(e['t'], e['old'], e['new'], e.get('why', ''))
            for e in events if e['kind'] == 'state']


def analyse(rec):
    cyc = rec['cycles']
    ev = rec['events']
    sc = state_changes(ev)
    out = {'ret': None, 'land': None, 'go': [], 'go_excluded': 0}
    commit = next((t for t, o, n, w in sc if n == 'COMMIT_LANDING'), None)
    landed = next((t for t, o, n, w in sc if n == 'LANDED'), None)
    rets = [e for e in ev if e['kind'] == 'return'
            and (commit is None or e['t'] <= commit)]
    if commit is not None and rets:
        last = rets[-1]
        # tau charged by C1 at that moment: overhead_s of the cycle
        c0 = min(cyc, key=lambda c: abs(c['t'] - last['t']))
        tau = c0['overhead_s'] or 0.0
        meas = consumed_at(cyc, commit) - consumed_at(cyc, last['t'])
        timeout = any(e['kind'] == 'return_timeout' for e in ev)
        out['ret'] = {'pred': last['energy_j'], 'tau_j': tau * P_HOVER,
                      'meas': meas, 'length_m': last['length_m'],
                      'n_legs': len(rets), 'timeout': timeout}
    if commit is not None and landed is not None:
        out['land'] = consumed_at(cyc, landed) - consumed_at(cyc, commit)
    # frontier transits: the cycle of each transition into NAVIGATING (the
    # candidates are re-evaluated and logged at every cycle while
    # navigating, so only the cycle of the transition holds the e_go the
    # decision used) -> next state change out of NAVIGATING
    ts = np.array([c['t'] for c in cyc])
    for t0, o, n, w in sc:
        if n != 'NAVIGATING':
            continue
        # the cycle stamp is the map stamp; the event follows it by the
        # decision time (up to about 0.1 s on W3): take the last cycle
        # stamped before the event
        before = np.nonzero(ts <= t0)[0]
        if before.size == 0:
            out['go_excluded'] += 1
            continue
        c = cyc[int(before[-1])]
        chosen = [x for x in c['candidates'] if x['chosen']]
        if len(chosen) != 1 or t0 - c['t'] > 0.5:
            out['go_excluded'] += 1
            continue
        nxt = next(((t, o, n, w) for t, o, n, w in sc
                    if t > t0 and o == 'NAVIGATING'), None)
        if nxt is None:
            continue
        if nxt[2] == 'SELECTING' and nxt[3] == 'arrived':
            meas = consumed_at(cyc, nxt[0]) - consumed_at(cyc, t0)
            out['go'].append({'pred': chosen[0]['e_go'], 'meas': meas,
                              'dist': chosen[0]['path_dist_m'],
                              'tau_j': (c['overhead_s'] or 0.0) * P_HOVER})
        else:
            out['go_excluded'] += 1
    return out


def summary(name, pred, meas):
    pred, meas = np.asarray(pred), np.asarray(meas)
    err = meas - pred
    rel = err / pred * 100
    print(f'  {name}: n {len(pred)}; measured - predicted: median '
          f'{np.median(err):+.0f} J [{np.percentile(err, 25):+.0f}, '
          f'{np.percentile(err, 75):+.0f}], min {err.min():+.0f}, max '
          f'{err.max():+.0f}; relative median {np.median(rel):+.1f} % '
          f'[{np.percentile(rel, 25):+.1f}, {np.percentile(rel, 75):+.1f}], '
          f'p95 {np.percentile(rel, 95):+.1f} %; under-predicted '
          f'(meas > pred) {np.mean(err > 0) * 100:.0f} %')


def main():
    allret, allland, allgo = [], [], []
    excl = 0
    for world in CAMPAIGNS:
        R, L, G = [], [], []
        for cfg, r, d in runs(world):
            if cfg == 'B0':
                continue  # B0 lands where its budget ends, no return leg
            a = analyse(load(d))
            if a['ret'] and not a['ret']['timeout']:
                R.append(a['ret'] | {'world': world, 'cfg': cfg, 'r': r})
            if a['land'] is not None:
                L.append(a['land'])
            G += a['go']
            excl += a['go_excluded']
        print(f'\n== {world}')
        summary('E_ret path only      ', [x['pred'] for x in R],
                [x['meas'] for x in R])
        summary('E_ret + tau (as C1)  ', [x['pred'] + x['tau_j'] for x in R],
                [x['meas'] for x in R])
        summary('E_land (3387 J plan) ', [E_LAND_PLAN] * len(L), L)
        summary('E_go (as C1, w/ tau) ', [x['pred'] for x in G],
                [x['meas'] for x in G])
        allret += R
        allland += L
        allgo += G
    print('\n== all worlds')
    summary('E_ret path only      ', [x['pred'] for x in allret],
            [x['meas'] for x in allret])
    summary('E_ret + tau (as C1)  ', [x['pred'] + x['tau_j'] for x in allret],
            [x['meas'] for x in allret])
    summary('E_ret+tau+E_land     ',
            [x['pred'] + x['tau_j'] + E_LAND_PLAN for x in allret],
            [x['meas'] + l for x, l in zip(allret, allland)]
            if len(allret) == len(allland) else [np.nan] * len(allret))
    summary('E_land (3387 J plan) ', [E_LAND_PLAN] * len(allland), allland)
    summary('E_go (as C1, w/ tau) ', [x['pred'] for x in allgo],
            [x['meas'] for x in allgo])
    summary('E_go path only       ', [x['pred'] - x['tau_j'] for x in allgo],
            [x['meas'] for x in allgo])
    under = [x for x in allgo if x['meas'] > x['pred']]
    print(f'  E_go under-predicted by C1: {len(under)} of {len(allgo)}, '
          f'by {[round(x["meas"] - x["pred"]) for x in under]} J')
    print(f'  E_land measured: {fmt(allland, 0)} J, max {max(allland):.0f}')
    print(f'  transits excluded (timeout, blacklist, abort, return): {excl}')
    worst = sorted(allret, key=lambda x: x['meas'] - x['pred'] - x['tau_j'])
    print('  worst E_ret+tau under-predictions:')
    for x in worst[-5:]:
        print(f"    {x['world']} {x['cfg']}_r{x['r']:02d}: pred "
              f"{x['pred']:.0f} + tau {x['tau_j']:.0f}, meas {x['meas']:.0f},"
              f" length {x['length_m']:.0f} m, legs {x['n_legs']}")
    np.savez('b1_energy_prediction.npz',
             ret_pred=[x['pred'] for x in allret],
             ret_tau=[x['tau_j'] for x in allret],
             ret_meas=[x['meas'] for x in allret],
             ret_world=[x['world'] for x in allret],
             ret_len=[x['length_m'] for x in allret],
             land=allland, go_pred=[x['pred'] for x in allgo],
             go_meas=[x['meas'] for x in allgo],
             go_dist=[x['dist'] for x in allgo])


if __name__ == '__main__':
    main()
