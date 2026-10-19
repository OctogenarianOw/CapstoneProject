"""
Unit tests for trajectory_profile, and its agreement with energy_model.

The energy model is valid only for the profile it was calibrated on, so
the planned durations of the two modules must be the same numbers.
"""

import math

import numpy as np
import pytest
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.trajectory_profile import TrapezoidPath

ZIGZAG = [(0, 0), (40, 0), (40, 10), (0, 10), (0, 20), (40, 20), (40, 30),
          (0, 30), (0, 0)]


def sample(path, dt=0.001):
    """Return times, positions, velocities and accelerations."""
    ts = np.arange(0.0, path.duration + 0.5, dt)
    p, v, a = zip(*(path.at(t) for t in ts))
    return ts, np.array(p), np.array(v), np.array(a)


@pytest.mark.parametrize('side', [10.0, 20.0, 40.0, 60.0])
def test_square_length_speed_and_end(side):
    """Exact length, peak 5 m/s and 3 m/s2, velocity = dp/dt, ends home."""
    pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    path = TrapezoidPath(pts, 5.0, 3.0)
    ts, p, v, a = sample(path)
    assert np.sum(np.hypot(*np.diff(p, axis=0).T)) == pytest.approx(
        4 * side, rel=1e-4)
    assert np.hypot(*v.T).max() == pytest.approx(5.0)
    assert np.hypot(*a.T).max() == pytest.approx(3.0)
    numeric = np.hypot(*(np.diff(p, axis=0) / 0.001).T)
    assert np.max(np.abs(np.hypot(*v[1:].T) - numeric)) < 0.01
    assert tuple(p[-1]) == (0.0, 0.0)


def test_durations_match_the_energy_model():
    """Profile duration equals the model's planned time minus settling."""
    for pts in (ZIGZAG, [(0, 0), (4, 0)], [(0, 0), (3, 4), (3, 4), (9, 12)]):
        path = TrapezoidPath(pts, MODEL.cruise_speed_ms, MODEL.accel_ms2)
        assert path.duration == pytest.approx(
            MODEL.path_time(pts) - MODEL.settle_s)


def test_leg_index_and_rest_at_vertices():
    """Speed is zero at every vertex, leg index follows the time."""
    path = TrapezoidPath(ZIGZAG, 5.0, 3.0)
    for k, leg in enumerate(path.legs):
        assert path.leg_index(leg['t0'] + 0.01) == k
        _, v, _ = path.at(leg['t0'])
        assert math.hypot(*v) == pytest.approx(0.0, abs=1e-9)
    assert path.at(path.duration + 5.0) == ((0, 0), (0.0, 0.0), (0.0, 0.0))
