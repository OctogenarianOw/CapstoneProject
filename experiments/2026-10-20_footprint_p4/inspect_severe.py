#!/usr/bin/env python3
"""
Where the severe false accepts of measure.py lie, and how often.

For each map and footprint, every severe cell with its truth zone, the
truth and map slope and step, and its observation count; then the cells
grouped by position over all mission maps, to tell a systematic place
(the same cell in many flights) from scattered map noise.

    cd ~/ros2_ws/droneAfter
    python3 PATH/inspect_severe.py data_survey_v6.npz \
        --runs experiments/2026-10-19_campaign_w1/runs
"""

import argparse
from collections import defaultdict
from pathlib import Path

import numpy as np

import measure as m
from measure import v, detect, classify, OBSERVED, VAR_MAX


def severe_cells(d, footprint):
    state = m.classify(d['sub_count'], d['sub_var'])
    truth = detect(d['truth_layers'], d['truth_ned'], d['res'],
                   d['sub_origin'], footprint_m=footprint, **v.THRESHOLDS)
    found = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=footprint,
                   variance=np.where(state == OBSERVED, d['sub_var'],
                                     np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    severe, _, _, _ = v.severity(d, found['candidate'] & ~truth['candidate'],
                                 2.0)
    half_t, res = d['half_t'], d['res']
    out = []
    for i, j in zip(*np.nonzero(severe)):
        code = int(d['zone_ned'][i, j])
        name = d['names'][code] if 0 <= code < len(d['names']) else str(code)
        out.append(dict(
            ij=(int(i), int(j)),
            north=-half_t + (i + 0.5) * res, east=-half_t + (j + 0.5) * res,
            zone=name,
            t_slope=float(d['truth_layers']['slope'][i, j]),
            t_step=float(d['truth_layers']['step'][i, j]),
            m_slope=float(d['map_layers']['slope'][i, j]),
            m_step=float(d['map_layers']['step'][i, j]),
            count=int(d['sub_count'][i, j])))
    return out


def line(label, fp, c):
    return (f'{label:<20} {fp:>4.1f} {c["north"]:>7.1f} {c["east"]:>7.1f} '
            f'{c["zone"]:<9} {c["t_slope"]:>6.1f} {c["t_step"]:>6.2f} '
            f'{c["m_slope"]:>6.1f} {c["m_step"]:>6.2f} {c["count"]:>6}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('maps', nargs='*')
    p.add_argument('--runs')
    args = p.parse_args()

    print(f'{"map":<20} {"fp":>4} {"N":>7} {"E":>7} {"zone":<9} '
          f'{"t.slp":>6} {"t.stp":>6} {"m.slp":>6} {"m.stp":>6} {"obs":>6}')
    for path in args.maps:
        d = m.load_map(path)
        for fp in m.FOOTPRINTS:
            for c in severe_cells(d, fp):
                print(line(Path(path).name, fp, c))

    if not args.runs:
        return
    where = {fp: defaultdict(list) for fp in m.FOOTPRINTS}
    for name, snap in m.last_snapshots(args.runs):
        d = m.load_map(snap)
        for fp in m.FOOTPRINTS:
            for c in severe_cells(d, fp):
                print(line(name, fp, c))
                where[fp][(c['north'], c['east'], c['zone'])].append(name)

    for fp in m.FOOTPRINTS:
        print()
        print(f'footprint {fp} m: severe cells by position over the missions')
        for (north, east, zone), names in sorted(
                where[fp].items(), key=lambda kv: -len(kv[1])):
            print(f'  N {north:+6.1f} E {east:+6.1f} {zone:<9} '
                  f'{len(names):>3} maps')


if __name__ == '__main__':
    main()
