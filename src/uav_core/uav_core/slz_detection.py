"""Safe landing zone detection from terrain layers.

Two stages, deliberately separate:

  cell test   each cell passes or fails the geometric thresholds
  zone test   a cell becomes a candidate only if its whole footprint
              passes, so a single good cell surrounded by bad ground
              is never offered as a landing site

The footprint is what the aircraft actually occupies plus a margin,
not one grid cell. That distinction is what makes an isolated bad
cell harmless: it fails its own test and also disqualifies every
footprint containing it.

Thresholds are measured, not assumed. See tools/validate_terrain.py
for where each default comes from.
"""

import numpy as np


def classify_cells(layers, max_slope_deg=6.0, max_roughness_m=0.05,
                   max_step_m=0.24, variance=None, max_variance=None):
    """Per-cell pass/fail against the geometric thresholds.

    layers     : dict from terrain_analysis.analyse
    variance   : optional (H, W) map variance, used as a confidence gate
    max_variance : cells less certain than this are refused outright

    An invalid cell, meaning one whose analysis window touched
    unobserved ground, always fails. Unknown terrain is never landable.
    """
    ok = layers['valid'].copy()

    with np.errstate(invalid='ignore'):
        ok &= layers['slope'] <= max_slope_deg
        ok &= layers['roughness'] <= max_roughness_m
        ok &= layers['step'] <= max_step_m

    if variance is not None and max_variance is not None:
        ok &= np.isfinite(variance) & (variance <= max_variance)

    return ok


def footprint_ok(cell_ok, radius):
    """Cells whose entire footprint passes the cell test.

    Shifting the pass mask by every footprint offset and taking the
    logical AND leaves exactly the cells with no failure anywhere in
    their footprint.
    """
    out = cell_ok.copy()
    h, w = cell_ok.shape
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di == 0 and dj == 0:
                continue
            shifted = np.zeros_like(cell_ok)
            src_i = slice(max(0, -di), h - max(0, di))
            src_j = slice(max(0, -dj), w - max(0, dj))
            dst_i = slice(max(0, di), h - max(0, -di))
            dst_j = slice(max(0, dj), w - max(0, -dj))
            shifted[dst_i, dst_j] = cell_ok[src_i, src_j]
            out &= shifted
    return out


def dilate(mask, radius):
    """Grow a mask by radius cells in every direction.

    Used to push candidate zones away from refused ground. The map
    smooths the foot of a slope, so a footprint that only just clears
    the thresholds there may sit closer to real hazard than its
    geometry suggests. The margin buys back that smoothing.
    """
    if radius <= 0:
        return mask
    out = mask.copy()
    h, w = mask.shape
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di == 0 and dj == 0:
                continue
            shifted = np.zeros_like(mask)
            src_i = slice(max(0, -di), h - max(0, di))
            src_j = slice(max(0, -dj), w - max(0, dj))
            dst_i = slice(max(0, di), h - max(0, -di))
            dst_j = slice(max(0, dj), w - max(0, -dj))
            shifted[dst_i, dst_j] = mask[src_i, src_j]
            out |= shifted
    return out

def label_zones(candidate):
    """Group adjacent candidate cells into connected zones.

    Returns a label array (0 = no zone) and the number of zones.
    Flood fill with 8-connectivity, written out rather than pulled
    from scipy so the module has no dependency beyond numpy.
    """
    h, w = candidate.shape
    labels = np.zeros((h, w), dtype=np.int32)
    current = 0

    neighbours = [(di, dj)
                  for di in (-1, 0, 1) for dj in (-1, 0, 1)
                  if not (di == 0 and dj == 0)]

    for i in range(h):
        for j in range(w):
            if not candidate[i, j] or labels[i, j]:
                continue
            current += 1
            stack = [(i, j)]
            labels[i, j] = current
            while stack:
                ci, cj = stack.pop()
                for di, dj in neighbours:
                    ni, nj = ci + di, cj + dj
                    if (0 <= ni < h and 0 <= nj < w
                            and candidate[ni, nj] and not labels[ni, nj]):
                        labels[ni, nj] = current
                        stack.append((ni, nj))

    return labels, current


def zone_summary(labels, n_zones, elevation, resolution, origin_xy):
    """One record per zone: centroid, area, mean height, cell count."""
    ox, oy = origin_xy
    zones = []
    for k in range(1, n_zones + 1):
        sel = labels == k
        idx_i, idx_j = np.nonzero(sel)
        zones.append({
            'id': k,
            'cells': int(sel.sum()),
            'area_m2': float(sel.sum()) * resolution ** 2,
            'centre_x': float(ox + (idx_i.mean() + 0.5) * resolution),
            'centre_y': float(oy + (idx_j.mean() + 0.5) * resolution),
            'mean_z': float(np.nanmean(elevation[sel])),
        })
    zones.sort(key=lambda z: -z['area_m2'])
    return zones


def detect(layers, elevation, resolution, origin_xy,
           footprint_m=3.0, safety_margin_m=0.0, **thresholds):
    """Full pipeline: cell test, safety margin, footprint test, labelling.

    safety_margin_m grows the refused region before the footprint test,
    so candidates keep their distance from anything that failed.
    """
    # Cells spanned by the footprint, as an odd count centred on the
    # cell: 2*radius+1 <= footprint_m / resolution. A radius of 0 is
    # legitimate when the footprint fits in one cell, because the cell
    # test already checks the 3x3 analysis window around it.
    radius = int(max(0.0, (footprint_m / resolution - 1.0) / 2.0))
    
    margin = int(round(safety_margin_m / resolution))

    cell_ok = classify_cells(layers, **thresholds)

    if margin > 0:
        # Dilating the refusal, rather than eroding the acceptance,
        # keeps cells that were never observed out of the margin: they
        # already fail, and their neighbourhood becomes refused too.
        refused = dilate(~cell_ok, margin)
        cell_ok = cell_ok & ~refused

    candidate = footprint_ok(cell_ok, radius)
    labels, n = label_zones(candidate)
    zones = zone_summary(labels, n, elevation, resolution, origin_xy)

    return {
        'cell_ok': cell_ok,
        'candidate': candidate,
        'labels': labels,
        'zones': zones,
        'footprint_radius': radius,
        'margin_cells': margin,
    }