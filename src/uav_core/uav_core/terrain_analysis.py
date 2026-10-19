"""Geometric layers derived from an elevation map.

Three quantities per cell, each computed over a square window:

    slope      inclination of the best-fit plane, in degrees
    roughness  standard deviation of the residuals to that plane
    step       largest height difference to any neighbour

Slope and roughness are independent by construction: the plane is
removed before the dispersion is measured, so a smooth ramp reads as
high slope and low roughness, while a flat rubble field reads as low
slope and high roughness. A landing site needs both to be small.

Step catches what neither sees: a vertical lip between two otherwise
flat areas. Mean slope across it stays modest while the hazard is real.

Cells whose window touches unobserved ground are marked invalid rather
than computed from partial data. Unknown terrain must never be
declared landable.
"""

import numpy as np


def _window_stack(grid, radius):
    """Stack of shifted copies, one per cell in the window.

    Returns an array of shape (n_offsets, H, W) where entry k holds
    the grid shifted by offset k, NaN-padded at the borders. Working
    on the whole stack keeps the computation vectorised instead of
    looping over cells.
    """
    h, w = grid.shape
    offsets = [(di, dj)
               for di in range(-radius, radius + 1)
               for dj in range(-radius, radius + 1)]

    out = np.full((len(offsets), h, w), np.nan, dtype=np.float64)
    for k, (di, dj) in enumerate(offsets):
        src_i = slice(max(0, -di), h - max(0, di))
        src_j = slice(max(0, -dj), w - max(0, dj))
        dst_i = slice(max(0, di), h - max(0, -di))
        dst_j = slice(max(0, dj), w - max(0, -dj))
        out[k, dst_i, dst_j] = grid[src_i, src_j]

    return out, offsets


def analyse(elevation, count, resolution, radius=1):
    """Compute slope, roughness and step from an elevation grid.

    elevation : (H, W) height per cell, NED so z points down
    count     : (H, W) observations per cell, 0 means unobserved
    radius    : window half-width, 1 gives a 3x3 window

    Returns a dict of (H, W) arrays. Invalid cells hold NaN, and the
    boolean 'valid' layer says where the results mean anything.
    """
    known = count > 0
    heights = np.where(known, elevation.astype(np.float64), np.nan)

    stack, offsets = _window_stack(heights, radius)

    # A window is usable only if every cell in it was observed. A
    # partially unknown window could hide exactly the hazard that
    # matters, so it is rejected rather than estimated.
    valid = np.all(np.isfinite(stack), axis=0)

    # Local coordinates of each window cell, in metres, relative to
    # the centre. Same order as the stack.
    dx = np.array([di for di, _ in offsets], dtype=np.float64) * resolution
    dy = np.array([dj for _, dj in offsets], dtype=np.float64) * resolution

    # Least-squares plane z = a*x + b*y + c over the window. With a
    # regular grid the design matrix is the same everywhere, so the
    # normal equations reduce to these closed forms.
    n = len(offsets)
    sxx = np.sum(dx * dx)
    syy = np.sum(dy * dy)

    z_mean = np.nanmean(stack, axis=0)
    a = np.nansum(stack * dx[:, None, None], axis=0) / sxx
    b = np.nansum(stack * dy[:, None, None], axis=0) / syy
    c = z_mean

    fitted = (a[None, :, :] * dx[:, None, None]
              + b[None, :, :] * dy[:, None, None]
              + c[None, :, :])
    residual = stack - fitted

    slope = np.degrees(np.arctan(np.hypot(a, b)))
    roughness = np.sqrt(np.nansum(residual ** 2, axis=0) / n)

    centre = stack[len(offsets) // 2]
    diffs = np.abs(stack - centre[None, :, :])
    # Windows that are entirely unobserved would make nanmax warn and
    # return NaN anyway; the valid mask already excludes them.
    step = np.where(valid, np.nanmax(np.where(valid, diffs, 0.0), axis=0),
                    np.nan)

    nan = np.full_like(slope, np.nan)
    return {
        'slope': np.where(valid, slope, nan),
        'roughness': np.where(valid, roughness, nan),
        'step': np.where(valid, step, nan),
        'valid': valid,
    }