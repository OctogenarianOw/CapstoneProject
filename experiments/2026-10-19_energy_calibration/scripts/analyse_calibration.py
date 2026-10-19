#!/usr/bin/env python3
"""Fit and validate E = P_hover * t + k_move * d (spec S10).

Each flight is cut at the phase changes calibration_flight.py published:
climb, hover_pre, transit, hover_post, land. Per segment:

  E  integral of rotor shaft power, P = k_m * k_f * sum(w^3), over the
     rotor samples (trapezoid), the instrument validated on hover on
     17 Oct (285.9 W measured against 286.5 W predicted)
  t  duration on the probe's monotonic clock
  d  horizontal distance, integral of |v_xy| from the EKF velocity. The
     sum of position steps was rejected: at 50 Hz, millimetre jitter in
     x, y adds up to metres over a long hover, while velocity noise
     integrates to a fraction of a metre

The fit uses hover_pre, transit and hover_post of the calibration runs,
least squares without intercept, since a zero-length, zero-duration
segment costs nothing. Climb is excluded: it is a regime the model does
not cover and the explorer only climbs once. Landing gives E_land.

Consistency check required by S10: P_hover from the fit against the
285.9 W of the pure hover flight of 17 Oct, which is not in the fit.

Validation runs are predicted twice. With measured t and d, which tests
the law. With the plan only, d = path length and t = hold + post +
planned profile time + mean arrival settling from calibration, which is
what C1 will have to work with before flying. The planned profile time
is exact from the plan, see calibration_flight.leg_profile.

    python3 analyse_calibration.py RUNS_DIR
"""

import json
import sys
from pathlib import Path

import numpy as np
from scipy import stats

KF = 8.54858e-06
KM = 0.016
P_HOVER_17OCT = 285.9
FIT_SEGMENTS = ('hover_pre', 'transit', 'hover_post')
PHASE_BOUNDS = {
    'climb': ('CLIMB', 'HOVER_PRE'),
    'hover_pre': ('HOVER_PRE', 'TRANSIT'),
    'transit': ('TRANSIT', 'HOVER_POST'),
    'hover_post': ('HOVER_POST', 'LAND'),
    'land': ('LAND', 'LANDED'),
}


def leg_time(length, v, a):
    """Rest-to-rest trapezoid duration, as calibration_flight.leg_profile."""
    v_peak = min(v, np.sqrt(a * length))
    t_ramp = v_peak / a
    return 2.0 * t_ramp + (length - v_peak * t_ramp) / v_peak


def read_setup(run):
    setup = {}
    for line in (run / 'setup.txt').read_text().splitlines():
        key, _, value = line.partition(' ')
        if key in ('path', 'speed', 'hold', 'post', 'alt', 'accel'):
            setup[key] = value
    pts = [tuple(map(float, p.split(','))) for p in setup['path'].split(';')]
    legs = [float(np.hypot(b[0] - a[0], b[1] - a[1]))
            for a, b in zip(pts[:-1], pts[1:])]
    speed = float(setup['speed'])
    accel = float(setup.get('accel', 'nan'))
    planned = (sum(leg_time(L, speed, accel) for L in legs if L > 0)
               if np.isfinite(accel) else float('nan'))
    return dict(path=setup['path'], corners=len(pts) - 2,
                length=float(sum(legs)), speed=speed, accel=accel,
                planned_transit=float(planned),
                hold=float(setup['hold']), post=float(setup['post']))


def integral(t, y, a, b):
    """Trapezoid integral of samples y(t) over [a, b], ends interpolated."""
    inside = (t > a) & (t < b)
    tt = np.concatenate(([a], t[inside], [b]))
    yy = np.concatenate(([np.interp(a, t, y)], y[inside],
                         [np.interp(b, t, y)]))
    return float(np.trapz(yy, tt))


def load_run(run):
    r = np.genfromtxt(run / 'rotors.csv', delimiter=',', names=True)
    s = np.genfromtxt(run / 'state.csv', delimiter=',', names=True)
    phases = {}
    for line in (run / 'phases.csv').read_text().splitlines()[1:]:
        t, name = line.split(',')
        phases.setdefault(name, float(t))
    w = np.stack([r[f'w{i}'] for i in range(4)], axis=1)
    power = KM * KF * np.sum(w ** 3, axis=1)
    speed_xy = np.hypot(s['vx'], s['vy'])
    segments = {}
    for name, (p0, p1) in PHASE_BOUNDS.items():
        if p0 not in phases or p1 not in phases:
            continue
        a, b = phases[p0], phases[p1]
        seg = dict(t=b - a,
                   E=integral(r['t_wall'], power, a, b),
                   d=integral(s['t_wall'], speed_xy, a, b))
        sel = (s['t_wall'] > a) & (s['t_wall'] < b)
        seg['v_median'] = float(np.median(speed_xy[sel])) if sel.any() \
            else float('nan')
        seg['v_p95'] = float(np.percentile(speed_xy[sel], 95)) \
            if sel.any() else float('nan')
        segments[name] = seg
    return segments


def fit(rows):
    X = np.array([[r['t'], r['d']] for r in rows])
    y = np.array([r['E'] for r in rows])
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    dof = len(y) - 2
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.inv(X.T @ X)
    tq = stats.t.ppf(0.975, dof)
    corr_td = float(np.corrcoef(X[:, 0], X[:, 1])[0, 1])
    return dict(P_hover=float(beta[0]), k_move=float(beta[1]),
                ci_P=float(tq * np.sqrt(cov[0, 0])),
                ci_k=float(tq * np.sqrt(cov[1, 1])),
                cov=cov.tolist(), n=len(y), dof=dof, tq=float(tq),
                corr_t_d=corr_td,
                rel_resid=(resid / y).tolist())


def main():
    root = Path(sys.argv[1])
    runs = sorted(p for p in root.iterdir()
                  if p.is_dir() and (p / 'phases.csv').exists())
    data = {}
    for run in runs:
        data[run.name] = dict(setup=read_setup(run), seg=load_run(run))

    cal = {k: v for k, v in data.items() if k.startswith('cal_')}
    val = {k: v for k, v in data.items() if k.startswith('val_')}

    print('== segments')
    print(f'{"run":<18} {"segment":<11} {"t s":>7} {"d m":>7} {"E J":>9} '
          f'{"P W":>7} {"v med":>6} {"v p95":>6}')
    for name, run in data.items():
        for seg_name, seg in run['seg'].items():
            print(f'{name:<18} {seg_name:<11} {seg["t"]:>7.2f} '
                  f'{seg["d"]:>7.2f} {seg["E"]:>9.1f} '
                  f'{seg["E"] / seg["t"]:>7.1f} {seg["v_median"]:>6.2f} '
                  f'{seg["v_p95"]:>6.2f}')

    rows = [dict(run=n, segment=s, **r['seg'][s])
            for n, r in cal.items() for s in FIT_SEGMENTS if s in r['seg']]
    if len(rows) < 3:
        print('not enough calibration segments to fit')
        return
    f = fit(rows)
    speeds = {r['setup']['speed'] for r in cal.values()}
    v = speeds.pop() if len(speeds) == 1 else float('nan')
    var_keff = (f['cov'][0][0] / v ** 2 + f['cov'][1][1]
                + 2 * f['cov'][0][1] / v)
    k_eff = f['P_hover'] / v + f['k_move']
    ci_keff = f['tq'] * np.sqrt(var_keff)

    print()
    print(f'== fit on {f["n"]} segments of {len(cal)} calibration runs')
    print(f'P_hover  {f["P_hover"]:8.2f} W    +/- {f["ci_P"]:.2f} (95 %)')
    print(f'k_move   {f["k_move"]:8.3f} J/m  +/- {f["ci_k"]:.3f} (95 %)')
    print(f'k_eff    {k_eff:8.3f} J/m  +/- {ci_keff:.3f} (95 %) at '
          f'{v} m/s')
    print(f'corr(t, d) over fit rows {f["corr_t_d"]:+.3f}')
    rel = np.abs(np.array(f['rel_resid']))
    print(f'relative residuals  median {100 * np.median(rel):.2f} %  '
          f'max {100 * rel.max():.2f} %')
    for r, e in zip(rows, f['rel_resid']):
        print(f'  {r["run"]:<18} {r["segment"]:<11} {100 * e:+6.2f} %')

    dP = 100.0 * (f['P_hover'] - P_HOVER_17OCT) / P_HOVER_17OCT
    print()
    print(f'== consistency: P_hover {f["P_hover"]:.1f} W against '
          f'{P_HOVER_17OCT} W pure hover of 17 Oct, {dP:+.2f} %')
    hover_only = [r['E'] / r['t'] for r in rows if r['segment'] != 'transit']
    print(f'   hover segments alone: mean {np.mean(hover_only):.1f} W, '
          f'sd {np.std(hover_only, ddof=1):.1f} W, n {len(hover_only)}')

    overheads = [r['seg']['transit']['t'] - r['setup']['planned_transit']
                 for r in cal.values() if 'transit' in r['seg']]
    print(f'   arrival settling, measured transit t - planned profile t: '
          f'mean {np.mean(overheads):.2f} s, '
          f'sd {np.std(overheads, ddof=1):.2f} s, n {len(overheads)}')
    for (name, r), o in zip(cal.items(), overheads):
        tr, st = r['seg']['transit'], r['setup']
        print(f'     {name:<18} {o:5.2f} s, planned '
              f'{st["planned_transit"]:6.2f} s, flown {tr["d"]:6.1f} m '
              f'for {st["length"]:.0f} m planned '
              f'({100 * (tr["d"] / st["length"] - 1):+.1f} %)')

    lands = [r['seg']['land'] for r in cal.values() if 'land' in r['seg']]
    E_land = float(np.mean([s['E'] for s in lands]))
    sd_land = float(np.std([s['E'] for s in lands], ddof=1))
    print()
    print(f'== E_land  mean {E_land:.0f} J, sd {sd_land:.0f} J, '
          f'n {len(lands)}, duration mean '
          f'{np.mean([s["t"] for s in lands]):.1f} s')

    print()
    print('== validation, window hover_pre start to land start, plus land')
    results = []
    for name, r in val.items():
        seg, st = r['seg'], r['setup']
        win = [seg[s] for s in FIT_SEGMENTS if s in seg]
        E = sum(s['E'] for s in win)
        t = sum(s['t'] for s in win)
        d = sum(s['d'] for s in win)
        pred_meas = f['P_hover'] * t + f['k_move'] * d
        t_plan = (st['hold'] + st['post'] + st['planned_transit']
                  + float(np.mean(overheads)))
        pred_plan = f['P_hover'] * t_plan + f['k_move'] * st['length']
        E_tot = E + seg['land']['E'] if 'land' in seg else float('nan')
        err_meas = 100 * (pred_meas - E) / E
        err_plan = 100 * (pred_plan - E) / E
        err_tot = 100 * (pred_plan + E_land - E_tot) / E_tot
        print(f'{name:<18} measured {E:8.0f} J | from t, d {pred_meas:8.0f}'
              f' J {err_meas:+6.2f} % | from plan {pred_plan:8.0f} J '
              f'{err_plan:+6.2f} % | with land {err_tot:+6.2f} %')
        results.append(dict(run=name, E=E, t=t, d=d, pred_meas=pred_meas,
                            pred_plan=pred_plan, err_meas=err_meas,
                            err_plan=err_plan, err_with_land=err_tot))

    out = dict(fit=f, k_eff=k_eff, ci_k_eff=float(ci_keff), speed=v,
               P_hover_17oct=P_HOVER_17OCT, consistency_pct=dP,
               arrival_settling_s=overheads, E_land=E_land,
               E_land_sd=sd_land, validation=results,
               segments={n: r['seg'] for n, r in data.items()},
               setups={n: r['setup'] for n, r in data.items()})
    (root / 'results.json').write_text(json.dumps(out, indent=1) + '\n')
    print(f'\nwritten {root / "results.json"}')


if __name__ == '__main__':
    main()
