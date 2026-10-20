#!/usr/bin/env python3
"""
The last map of a W2 mission against the W2 truth, and the registry.

The evaluation of tools/validate_slz.py and 2026-10-20_min_zone, on the
W2 truth: the manager's detection (OBSERVED cells, footprint 1.5 m)
against the truth detected with the same chain, recall, false accepts
split marginal or severe (truth past twice a threshold, all of the
20 deg field), and the severe ones left in zones that hold a whole
footprint (the zones the registry may confirm). Then the zones the
registry confirmed during the flight, with the pad, if any, under each.

    python3 map_check.py RUN_DIR [RUN_DIR ...]
"""

import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))

from uav_core.observation import classify, OBSERVED, VAR_MAX  # noqa: E402
from uav_core.slz_detection import detect, footprint_ok  # noqa: E402
import validate_slz as v  # noqa: E402

TRUTH = ROOT / 'worlds' / 'w2_truth.npz'
PADS = {'home': (0.0, 0.0), 'pad_ne': (40.0, 48.0), 'pad_sw': (-48.0, -40.0)}
PAD_HALF = 8.0             # NED (north, east) centres, 16 m side


def load(snapshot):
    s = np.load(snapshot)
    buf = io.BytesIO()
    np.savez(buf, elevation=s['elevation'], variance=s['variance'],
             count=s['count'], resolution=s['resolution'],
             origin_x=s['origin_north'], origin_y=s['origin_east'])
    buf.seek(0)
    d = v.analyse_both(v.load(SimpleNamespace(
        map_file=buf, truth_file=str(TRUTH), truth_layer='truth')))
    res, n = d['res'], d['n_t']
    i0 = int(round((-d['half_t'] + res / 2.0 - float(s['origin_north']))
                   / res))
    j0 = int(round((-d['half_t'] + res / 2.0 - float(s['origin_east']))
                   / res))
    d['sub_var'] = s['variance'][i0:i0 + n, j0:j0 + n]
    return d


def pad_of(n, e):
    for name, (pn, pe) in PADS.items():
        if abs(n - pn) <= PAD_HALF and abs(e - pe) <= PAD_HALF:
            return name
    return 'field'


for arg in sys.argv[1:]:
    run = Path(arg)
    mission = next((run / 'missions').iterdir())
    snap = sorted((mission / 'maps').glob('map_*.npz'))[-1]
    d = load(snap)
    state = classify(d['sub_count'], d['sub_var'])
    truth = detect(d['truth_layers'], d['truth_ned'], d['res'],
                   d['sub_origin'], footprint_m=1.5, **v.THRESHOLDS)
    found = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=1.5,
                   variance=np.where(state == OBSERVED, d['sub_var'],
                                     np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    mapped, real = found['candidate'], truth['candidate']
    severe, benign, _, _ = v.severity(d, mapped & ~real, 2.0)
    core = footprint_ok(mapped, found['footprint_offsets'])
    held = np.isin(found['labels'],
                   np.unique(found['labels'][core])[1:]) & mapped
    print(f'{run.name}: last map {snap.name}')
    print(f'  truth landable {int(real.sum())}, map landable '
          f'{int(mapped.sum())}, recall '
          f'{100.0 * (mapped & real).sum() / max(1, real.sum()):.1f} %, '
          f'false accepts {int(benign.sum())} marginal, '
          f'{int(severe.sum())} severe, '
          f'{int((severe & held).sum())} severe in zones that hold a '
          f'footprint')
    cycles = [json.loads(x) for x in (mission / 'cycles.jsonl').open()]
    confirmed = {}
    for c in cycles:
        for z in c['zones']:
            if z['state'] == 'CONFIRMED' and not z['provisional']:
                confirmed.setdefault(z['id'], (c['t'], z['touchdown_ne'],
                                               z['area_m2']))
    for zid, (t, (n, e), area) in sorted(confirmed.items()):
        print(f'  zone {zid} confirmed at {t:6.1f} s, {area:5.0f} m2, '
              f'touchdown N {n:+6.1f} E {e:+6.1f}, on {pad_of(n, e)}')
    end = cycles[-1]['vehicle_ned']
    print(f'  final position N {end[0]:+6.1f} E {end[1]:+6.1f}, on '
          f'{pad_of(end[0], end[1])}')
