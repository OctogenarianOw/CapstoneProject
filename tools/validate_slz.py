"""Measure the safety of SLZ detection against ground truth.

Two error types, and they are not symmetric:

  false accept  a zone declared landable where the real terrain is not
                -> a destroyed aircraft. Severity is read from the truth
                   geometry of the cell, not from its zone label: a cell
                   just past the acceptance limit is a labelling artefact,
                   one far past it is a crash.
  false reject  safe ground declared unlandable
                -> lost opportunity. Tolerable, and the price of
                   refusing to decide on poorly observed terrain.

Truth runs through the identical pipeline, so any difference comes
from the map rather than from the thresholds.

    python3 tools/validate_slz.py data_survey_v4.npz
    python3 tools/validate_slz.py data_survey_v4.npz --margin 1.0
    python3 tools/validate_slz.py data_survey_v4.npz --sweep
"""

import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

sys.path.insert(0, '/home/doria/ros2_ws/droneAfter/src/uav_core')
from uav_core.terrain_analysis import analyse
from uav_core.slz_detection import detect

DEFAULT_MAP = '/tmp/elevation_map.npz'
DEFAULT_TRUTH = '/home/doria/ros2_ws/droneAfter/worlds/validation_truth.npz'

# Only used when the truth file predates zone_names being stored.
FALLBACK_ZONE_NAMES = ['flat', 'ramp_ok', 'ramp_bad', 'rough', 'obstacle']

THRESHOLDS = dict(max_slope_deg=6.0, max_roughness_m=0.05, max_step_m=0.24)


def parse_args():
    p = argparse.ArgumentParser(
        description='Compare detected landing zones against ground truth.')
    p.add_argument('map_file', nargs='?', default=DEFAULT_MAP,
                   help='saved elevation map, e.g. data_survey_v4.npz')
    p.add_argument('truth_file', nargs='?', default=DEFAULT_TRUTH)
    p.add_argument('--footprint', type=float, default=1.5,
                   help='required clear footprint, metres')
    p.add_argument('--margin', type=float, default=0.0,
                   help='extra dilation applied to the map only, metres')
    p.add_argument('--truth-layer', default='truth',
                   help='truth, truth_mean or truth_max. truth_max is the '
                        'safety reading: a cell holding a wall is as high '
                        'as the wall, not as high as its average')
    p.add_argument('--hazard-factor', type=float, default=2.0,
                   help='a false accept counts as severe when the truth '
                        'exceeds this multiple of an acceptance threshold')
    p.add_argument('--sweep', action='store_true',
                   help='run every footprint and margin combination and '
                        'print one table, instead of one detailed report')
    p.add_argument('--footprints', default='1.5,3.0,5.0')
    p.add_argument('--margins', default='0.0,1.0,2.0')
    p.add_argument('--plot', default='/tmp/slz.png')
    return p.parse_args()


# ------------------------------------------------------------------ load


def load(args):
    m = np.load(args.map_file)
    elev = m['elevation']
    count = m['count']
    res = float(m['resolution'])
    ox, oy = float(m['origin_x']), float(m['origin_y'])

    t = np.load(args.truth_file)
    layer = args.truth_layer if args.truth_layer in t.files else 'truth'
    truth_ned = -t[layer].T            # ENU up to NED down, axes swapped
    zone_ned = t['zone'].T

    if 'zone_names' in t.files:
        names = [str(s) for s in t['zone_names']]
    else:
        names = FALLBACK_ZONE_NAMES

    n_t = truth_ned.shape[0]
    half_t = n_t * res / 2.0
    i0 = int(round((-half_t + res / 2.0 - ox) / res))
    j0 = int(round((-half_t + res / 2.0 - oy) / res))

    if i0 < 0 or j0 < 0 or i0 + n_t > elev.shape[0] or j0 + n_t > elev.shape[1]:
        raise SystemExit('truth grid does not fit inside the map')

    sub_elev = elev[i0:i0 + n_t, j0:j0 + n_t]
    sub_count = count[i0:i0 + n_t, j0:j0 + n_t]

    return dict(res=res, half_t=half_t, n_t=n_t, layer=layer,
                sub_elev=sub_elev, sub_count=sub_count,
                sub_origin=(-half_t, -half_t),
                truth_ned=truth_ned, zone_ned=zone_ned, names=names,
                known=sub_count > 0)


def analyse_both(d):
    d['map_layers'] = analyse(d['sub_elev'], d['sub_count'], d['res'])
    d['truth_layers'] = analyse(d['truth_ned'],
                                np.ones_like(d['sub_count']), d['res'])
    return d


def run(d, footprint, margin):
    """Detect on the map and on the truth for one configuration."""
    map_res = detect(d['map_layers'], d['sub_elev'], d['res'], d['sub_origin'],
                     footprint_m=footprint,
                     safety_margin_m=margin, **THRESHOLDS)

    # The truth gets no margin: it defines what is genuinely safe, and the
    # margin is a precaution only an imperfect map needs to take. It does
    # get the footprint, because whether the aircraft fits is a fact about
    # the terrain, not about the map.
    truth_res = detect(d['truth_layers'], d['truth_ned'], d['res'],
                       d['sub_origin'], footprint_m=footprint,
                       safety_margin_m=0.0, **THRESHOLDS)

    mapped = map_res['candidate']
    real = truth_res['candidate']
    return dict(map_res=map_res, truth_res=truth_res,
                mapped=mapped, real=real,
                correct=mapped & real,
                false_accept=mapped & ~real,
                false_reject=real & ~mapped)


def severity(d, false_accept, factor):
    """Split false accepts by how far the real terrain is past the limits.

    Zone labels are not used: a gentle ramp is safe ground whatever it is
    called, and a cliff edge inside a zone named flat is not.
    """
    slope_limit = factor * THRESHOLDS['max_slope_deg']
    step_limit = factor * THRESHOLDS['max_step_m']
    severe = ((d['truth_layers']['slope'] > slope_limit)
              | (d['truth_layers']['step'] > step_limit))
    return false_accept & severe, false_accept & ~severe, slope_limit, step_limit


# ---------------------------------------------------------------- sweep


def sweep(d, args):
    footprints = [float(v) for v in args.footprints.split(',')]
    margins = [float(v) for v in args.margins.split(',')]

    print(f'map          {args.map_file}')
    print(f'truth layer  {d["layer"]}')
    print(f'thresholds   slope<={THRESHOLDS["max_slope_deg"]}deg '
          f'roughness<={THRESHOLDS["max_roughness_m"]}m '
          f'step<={THRESHOLDS["max_step_m"]}m')
    print()
    print(f'{"fprint":>7} {"margin":>7} {"cells":>6} {"truth ok":>9} '
          f'{"map ok":>8} {"recall":>7} {"f.acc":>7} {"marginal":>9} '
          f'{"severe":>7}')

    for footprint in footprints:
        for margin in margins:
            r = run(d, footprint, margin)
            sev, marg, _, _ = severity(d, r['false_accept'],
                                       args.hazard_factor)
            real = r['real'].sum()
            recall = 100.0 * r['correct'].sum() / real if real else 0.0
            side = 2 * r['map_res']['footprint_radius'] + 1
            print(f'{footprint:>7.1f} {margin:>7.1f} {side:>6} {real:>9} '
                  f'{r["mapped"].sum():>8} {recall:>6.1f}% '
                  f'{r["false_accept"].sum():>7} {marg.sum():>9} '
                  f'{sev.sum():>7}')

    print()
    print('severe is the column that must reach 0. Re-run the chosen pair')
    print('without --sweep for the per zone breakdown and the figure.')


# --------------------------------------------------------------- report


def report(d, args):
    r = run(d, args.footprint, args.margin)
    mapped, real = r['mapped'], r['real']
    correct = r['correct']
    false_accept, false_reject = r['false_accept'], r['false_reject']
    danger, benign, slope_limit, step_limit = severity(
        d, false_accept, args.hazard_factor)

    res, half_t, n_t = d['res'], d['half_t'], d['n_t']
    zone_ned, names = d['zone_ned'], d['names']
    truth_layers, map_layers = d['truth_layers'], d['map_layers']
    sub_elev, sub_count = d['sub_elev'], d['sub_count']

    def zone_name(code):
        return names[code] if 0 <= code < len(names) else f'zone{code}'

    print(f'map        {args.map_file}')
    print(f'truth      {args.truth_file} [{d["layer"]}]')
    print(f'thresholds slope<={THRESHOLDS["max_slope_deg"]}deg '
          f'roughness<={THRESHOLDS["max_roughness_m"]}m '
          f'step<={THRESHOLDS["max_step_m"]}m')
    print(f'footprint  {args.footprint} m '
          f'({2 * r["map_res"]["footprint_radius"] + 1} cells square)')
    print(f'margin     {args.margin} m ({r["map_res"]["margin_cells"]} cells)')
    print()

    print(f'truth says landable   {real.sum():>6} cells')
    print(f'map says landable     {mapped.sum():>6} cells')
    print(f'  correct             {correct.sum():>6}')
    print(f'  FALSE ACCEPT        {false_accept.sum():>6}')
    print(f'  false reject        {false_reject.sum():>6}')
    if real.sum():
        print(f'  recall              {100.0 * correct.sum() / real.sum():>5.1f}%')
    print()

    print(f'{"zone":<10} {"cells":>6} {"truth ok":>9} {"map ok":>8} '
          f'{"f.accept":>9} {"f.reject":>9} {"recall":>7}')
    for code, name in enumerate(names):
        sel = zone_ned == code
        zone_real = (real & sel).sum()
        zone_recall = (100.0 * (correct & sel).sum() / zone_real
                       if zone_real else float('nan'))
        print(f'{name:<10} {sel.sum():>6} {zone_real:>9} '
              f'{(mapped & sel).sum():>8} '
              f'{(false_accept & sel).sum():>9} '
              f'{(false_reject & sel).sum():>9} '
              f'{zone_recall:>6.1f}%')

    print()
    print(f'false accepts: {benign.sum()} marginal (truth within '
          f'{args.hazard_factor:g}x the limits), '
          f'{danger.sum()} severe  <- must be 0')
    print(f'  severe means truth slope > {slope_limit:.1f} deg '
          f'or truth step > {step_limit:.2f} m')
    if false_accept.any():
        print(f'  worst false accept: truth slope '
              f'{truth_layers["slope"][false_accept].max():.1f} deg, '
              f'truth step {truth_layers["step"][false_accept].max():.2f} m')

    if danger.sum():
        print()
        print(f'{"N":>7} {"E":>7} {"zone":<9} {"true":>8} {"map":>8} '
              f'{"step":>8} {"obs":>6}')
        ii, jj = np.nonzero(danger)
        order = np.argsort(-truth_layers['slope'][danger])
        for k in order[:20]:
            i, j = ii[k], jj[k]
            print(f'{-half_t + (i + 0.5) * res:>7.1f} '
                  f'{-half_t + (j + 0.5) * res:>7.1f} '
                  f'{zone_name(zone_ned[i, j]):<9} '
                  f'{truth_layers["slope"][i, j]:>7.1f}° '
                  f'{map_layers["slope"][i, j]:>7.1f}° '
                  f'{map_layers["step"][i, j]:>7.3f}m '
                  f'{sub_count[i, j]:>6}')

        # The slope figures disagree by tens of degrees, so the heights
        # themselves have to be inspected before designing any fix.
        ground = np.median(sub_elev[(zone_ned == 0) & d['known']])
        k = int(np.argmax(truth_layers['slope'][danger]))
        ci, cj = int(ii[k]), int(jj[k])
        lo_i, hi_i = max(0, ci - 2), min(n_t - 1, ci + 2)
        lo_j, hi_j = max(0, cj - 2), min(n_t - 1, cj + 2)
        print()
        print(f'heights around N {-half_t + (ci + 0.5) * res:+.1f} '
              f'E {-half_t + (cj + 0.5) * res:+.1f}, north up, '
              f'map offset {ground:+.3f} removed')
        for label, grid, fmt in (('truth', d['truth_ned'], '>9.2f'),
                                 ('map', sub_elev - ground, '>9.2f'),
                                 ('obs', sub_count, '>9.0f')):
            print(f'  {label}:')
            for i in range(hi_i, lo_i - 1, -1):
                print('    ' + ''.join(
                    f'{grid[i, j]:{fmt}}' for j in range(lo_j, hi_j + 1)))

    print()
    print(f'zones found in map: {len(r["map_res"]["zones"])}')
    for z in r['map_res']['zones'][:5]:
        print(f'  #{z["id"]:<3} {z["area_m2"]:>7.0f} m2 '
              f'at N {z["centre_x"]:+6.1f} E {z["centre_y"]:+6.1f}')

    plot(d, r, danger, benign, args)


# ------------------------------------------------------------------ plot


def plot(d, r, danger, benign, args):
    half_t = d['half_t']
    real, mapped = r['real'], r['mapped']
    fig, ax = plt.subplots(1, 3, figsize=(18, 6))
    extent = [-half_t, half_t, -half_t, half_t]

    truth_cmap = ListedColormap(['#e8e8e8', '#2a7a3a'])
    ax[0].imshow(real, origin='lower', extent=extent, cmap=truth_cmap,
                 vmin=0, vmax=1)
    ax[0].set_title('Ground truth')
    ax[0].legend(handles=[Patch(facecolor='#2a7a3a', label='landable'),
                          Patch(facecolor='#e8e8e8', label='not landable')],
                 loc='lower left', fontsize=8)

    map_cmap = ListedColormap(['#e8e8e8', '#1f4e79'])
    ax[1].imshow(mapped, origin='lower', extent=extent, cmap=map_cmap,
                 vmin=0, vmax=1)
    ax[1].set_title(f'Detected from map '
                    f'(footprint {args.footprint} m, margin {args.margin} m)')
    ax[1].legend(handles=[Patch(facecolor='#1f4e79', label='landable'),
                          Patch(facecolor='#e8e8e8', label='not landable')],
                 loc='lower left', fontsize=8)

    overlay = np.zeros(real.shape + (3,))
    overlay[...] = [0.10, 0.10, 0.10]
    overlay[r['correct']] = [0.20, 0.70, 0.30]
    overlay[r['false_reject']] = [1.00, 0.70, 0.20]
    overlay[benign] = [1.00, 0.85, 0.00]
    overlay[danger] = [1.00, 0.00, 0.00]
    ax[2].imshow(overlay, origin='lower', extent=extent)
    ax[2].set_title('Agreement')
    ax[2].legend(handles=[
        Patch(facecolor=(0.20, 0.70, 0.30), label='correct'),
        Patch(facecolor=(1.00, 0.70, 0.20), label='false reject (safe)'),
        Patch(facecolor=(1.00, 0.85, 0.00), label='false accept, marginal'),
        Patch(facecolor=(1.00, 0.00, 0.00), label='FALSE ACCEPT, severe'),
        Patch(facecolor=(0.10, 0.10, 0.10), label='refused by both'),
    ], loc='lower left', fontsize=8)

    for a in ax:
        a.set_xlabel('y east (m)')
        a.set_ylabel('x north (m)')

    plt.tight_layout()
    plt.savefig(args.plot, dpi=110)
    print(f'\nwrote {args.plot}')


def main():
    args = parse_args()
    d = analyse_both(load(args))
    if args.sweep:
        sweep(d, args)
    else:
        report(d, args)


if __name__ == '__main__':
    main()