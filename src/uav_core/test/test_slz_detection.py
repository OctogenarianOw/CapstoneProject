"""
Unit tests for the footprint element of slz_detection.

Two properties, both checked without ROS or simulation. The element must
be exactly the set of cells the footprint disc overlaps, found here by
dense sampling of the disc. And the smallest isolated zone the detector
accepts must match W >= (2 * (ra + margin + rf) + 1) * res, the formula
the thesis uses to state the minimum detectable landing zone.
"""

import numpy as np
import pytest

from uav_core.slz_detection import detect, footprint_offsets
from uav_core.terrain_analysis import analyse

THRESHOLDS = {'max_slope_deg': 6.0, 'max_roughness_m': 0.05,
              'max_step_m': 0.24}


def overlapped_cells(footprint_m, resolution, step_fraction=0.01):
    """Cells holding at least one sample strictly inside the disc."""
    r = footprint_m / 2.0
    step = resolution * step_fraction
    axis = np.arange(-r, r + step, step)
    x, y = np.meshgrid(axis, axis, indexing='ij')
    inside = x * x + y * y < r * r
    i = np.floor(x[inside] / resolution + 0.5).astype(int)
    j = np.floor(y[inside] / resolution + 0.5).astype(int)
    return set(zip(i.tolist(), j.tolist())) | {(0, 0)}


@pytest.mark.parametrize('footprint_m, resolution', [
    (0.5, 1.0), (1.5, 1.0), (3.0, 1.0), (5.0, 1.0), (7.0, 1.0),
    (1.5, 0.5), (3.0, 0.5),
])
def test_offsets_are_the_overlapped_cells(footprint_m, resolution):
    assert (set(footprint_offsets(footprint_m, resolution))
            == overlapped_cells(footprint_m, resolution))


def smallest_accepted_width(resolution, footprint_m, margin_m, radius):
    """Side of the smallest flat square found inside a refused checkerboard."""
    for w in range(1, 40):
        n = w + 20
        ii, jj = np.indices((n, n))
        elevation = -1.0 * ((ii + jj) % 2)
        a = (n - w) // 2
        elevation[a:a + w, a:a + w] = 0.0
        layers = analyse(elevation, np.ones((n, n), dtype=np.int32),
                         resolution, radius=radius)
        out = detect(layers, elevation, resolution, (0.0, 0.0),
                     footprint_m=footprint_m, safety_margin_m=margin_m,
                     **THRESHOLDS)
        if out['candidate'].any():
            return w * resolution
    return None


@pytest.mark.parametrize('resolution, radius, footprint_m, margin_m, width', [
    (1.0, 1, 5.0, 1.0, 9.0),
    (1.0, 1, 5.0, 0.0, 7.0),
    (1.0, 1, 3.0, 0.0, 5.0),
    (1.0, 1, 1.5, 0.0, 5.0),
    (0.5, 2, 1.5, 0.0, 3.5),
])
def test_minimum_detectable_width(resolution, radius, footprint_m,
                                  margin_m, width):
    assert smallest_accepted_width(resolution, footprint_m, margin_m,
                                   radius) == pytest.approx(width)
