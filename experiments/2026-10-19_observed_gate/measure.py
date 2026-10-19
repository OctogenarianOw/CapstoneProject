#!/usr/bin/env python3
"""
Effect of restricting landing classification to OBSERVED cells (S8, I4).

Same evaluation as tools/validate_slz.py (truth detected with the same
footprint, severe = truth past twice a threshold), map detection run
twice: as committed, and with the variance gate of classify_cells fed
with OBSERVED cells only (variance set to +inf elsewhere, max_variance =
observation.VAR_MAX), which is what the manager will do.

    python3 measure.py MAP.npz [MAP.npz ...]
"""

from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))

from uav_core.observation import classify, OBSERVED, VAR_MAX  # noqa: E402
from uav_core.slz_detection import detect  # noqa: E402
import validate_slz as v  # noqa: E402


def evaluate(d, map_res, truth_res):
    mapped, real = map_res['candidate'], truth_res['candidate']
    fa = mapped & ~real
    severe, benign, _, _ = v.severity(d, fa, 2.0)
    return (100.0 * (mapped & real).sum() / real.sum(), int(mapped.sum()),
            int(benign.sum()), int(severe.sum()))


print(f'{"map":<22} {"gate":<9} {"recall":>7} {"accepted":>9} '
      f'{"benign":>7} {"severe":>7}')
for path in sys.argv[1:]:
    args = SimpleNamespace(map_file=path, truth_file=v.DEFAULT_TRUTH,
                           truth_layer='truth')
    d = v.analyse_both(v.load(args))
    m = np.load(path)
    i0 = int(round((-d['half_t'] + d['res'] / 2.0 - float(m['origin_x']))
                   / d['res']))
    j0 = int(round((-d['half_t'] + d['res'] / 2.0 - float(m['origin_y']))
                   / d['res']))
    n = d['n_t']
    var = m['variance'][i0:i0 + n, j0:j0 + n]
    state = classify(d['sub_count'], var)
    truth_res = detect(d['truth_layers'], d['truth_ned'], d['res'],
                       d['sub_origin'], footprint_m=1.5, **v.THRESHOLDS)
    plain = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=1.5, **v.THRESHOLDS)
    gated = detect(d['map_layers'], d['sub_elev'], d['res'],
                   d['sub_origin'], footprint_m=1.5,
                   variance=np.where(state == OBSERVED, var, np.inf),
                   max_variance=VAR_MAX, **v.THRESHOLDS)
    for label, res in (('none', plain), ('OBSERVED', gated)):
        r, acc, ben, sev = evaluate(d, res, truth_res)
        print(f'{Path(path).name:<22} {label:<9} {r:6.1f}% {acc:9d} '
              f'{ben:7d} {sev:7d}')
