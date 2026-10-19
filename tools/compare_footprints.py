#!/usr/bin/env python3
"""Compare footprint structuring elements against ground truth.

Measures the choice made in slz_detection.footprint_offsets. Three
elements go through the identical pipeline, on the map and on the truth:

  square   largest odd square inside the disc, the element used before
           17 Oct 2026, kept here only as the rejected baseline
  centre   same square half-width, offsets kept when their centre lies
           within it, the circular fix first proposed in the spec
  cover    every cell the footprint disc overlaps, the retained element

The severe column is the one that matters: a false accept whose truth is
more than twice past a threshold. It must be 0.

    python3 tools/compare_footprints.py data_survey_v6.npz
    python3 tools/compare_footprints.py data_survey_v*.npz --shapes
"""

import argparse
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'src' / 'uav_core'))

from uav_core.slz_detection import (                          # noqa: E402
    classify_cells, dilate, footprint_offsets, footprint_ok)
import validate_slz as v                                      # noqa: E402


def offsets_square(footprint_m, resolution):
    r = int(max(0.0, (footprint_m / resolution - 1.0) / 2.0))
    return [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1)]


def offsets_centre(footprint_m, resolution):
    r = int(max(0.0, (footprint_m / resolution - 1.0) / 2.0))
    return [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1)
            if i * i + j * j <= r * r]


ELEMENTS = {
    'square': offsets_square,
    'centre': offsets_centre,
    'cover': footprint_offsets,
}


def candidate(layers, offsets, margin_cells):
    """Run the steps of slz_detection.detect with a given element."""
    ok = classify_cells(layers, **v.THRESHOLDS)
    if margin_cells > 0:
        ok = ok & ~dilate(~ok, margin_cells)
    return footprint_ok(ok, offsets)


def draw(offsets):
    n = max(max(abs(i), abs(j)) for i, j in offsets)
    cells = set(offsets)
    return [' '.join('#' if (i, j) in cells else '.'
                     for j in range(-n, n + 1))
            for i in range(n, -n - 1, -1)]


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('maps', nargs='+')
    p.add_argument('--truth', default=v.DEFAULT_TRUTH)
    p.add_argument('--footprints', default='1.5,3.0,5.0')
    p.add_argument('--margins', default='0.0,1.0')
    p.add_argument('--shapes', action='store_true',
                   help='also print each element at 1 m resolution')
    a = p.parse_args()
    footprints = [float(x) for x in a.footprints.split(',')]
    margins = [float(x) for x in a.margins.split(',')]

    if a.shapes:
        for fp in footprints:
            print(f'footprint {fp} m')
            for name, fn in ELEMENTS.items():
                offs = fn(fp, 1.0)
                print(f'  {name} ({len(offs)} cells)')
                for row in draw(offs):
                    print('    ' + row)
            print()

    print(f'{"map":<22} {"fp":>4} {"mrg":>4} {"element":<7} {"cells":>5} '
          f'{"truth ok":>8} {"map ok":>7} {"recall":>7} {"f.acc":>6} '
          f'{"marg":>5} {"severe":>6}')
    for path in a.maps:
        args = SimpleNamespace(map_file=path, truth_file=a.truth,
                               truth_layer='truth', hazard_factor=2.0)
        d = v.analyse_both(v.load(args))
        name_map = Path(path).name
        for fp in footprints:
            for mg in margins:
                mc = int(round(mg / d['res']))
                for name, fn in ELEMENTS.items():
                    offs = fn(fp, d['res'])
                    mapped = candidate(d['map_layers'], offs, mc)
                    # The truth never gets a margin: it defines what is
                    # safe. It does get the footprint, see validate_slz.
                    real = candidate(d['truth_layers'], offs, 0)
                    fa = mapped & ~real
                    severe, marginal, _, _ = v.severity(d, fa, 2.0)
                    recall = 100.0 * (mapped & real).sum() / max(1, real.sum())
                    print(f'{name_map:<22} {fp:4.1f} {mg:4.1f} {name:<7} '
                          f'{len(offs):5d} {real.sum():8d} {mapped.sum():7d} '
                          f'{recall:6.1f}% {fa.sum():6d} {marginal.sum():5d} '
                          f'{severe.sum():6d}')
            print()


if __name__ == '__main__':
    main()
