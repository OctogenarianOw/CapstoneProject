"""
Unit tests for energy_model, without ROS or simulation.

The validation flights of 19 Oct were never used in the fit, so
predicting them from their plan alone, as C1 will have to, is the
check that matters. Measured energies come from
experiments/2026-10-19_energy_calibration/runs/results.json.
"""

import pytest
from uav_core.energy_model import CALIBRATION_2026_10_19 as M


def square(side):
    """Return the closed square flown in calibration."""
    return [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]


ZIGZAG = [(0, 0), (40, 0), (40, 10), (0, 10), (0, 20), (40, 20), (40, 30),
          (0, 30), (0, 0)]


@pytest.mark.parametrize('length, expected', [
    (10.0, 2 * 5 / 3 + (10 - 25 / 3) / 5),   # reaches 5 m/s
    (25 / 3, 2 * 5 / 3),                     # just reaches 5 m/s
    (4.0, 2 * (12 ** 0.5) / 3),              # triangle, peak sqrt(a L)
    (0.0, 0.0),
])
def test_leg_time(length, expected):
    """Trapezoid and triangle durations."""
    assert M.leg_time(length) == pytest.approx(expected)


def test_path_time_matches_calibration_profile():
    """Planned transit times of the flight node, plus the mean settling."""
    assert M.path_time(square(10)) == pytest.approx(14.6667 + M.settle_s,
                                                    abs=1e-3)
    assert M.path_time(ZIGZAG) == pytest.approx(57.3333 + M.settle_s,
                                                abs=1e-3)


@pytest.mark.parametrize('path, hold, measured', [
    (square(30), 60.0, 27625.8),
    (square(30), 60.0, 27628.4),
    (ZIGZAG, 10.0, 21139.1),
    (ZIGZAG, 10.0, 21140.3),
])
def test_validation_flights_from_plan(path, hold, measured):
    """Unfitted flights predicted from the plan within 0.5 %."""
    predicted = M.hover_energy(hold + 5.0) + M.path_energy(path)
    assert abs(predicted - measured) / measured < 0.005


def test_each_leg_costs_its_ramps():
    """Splitting a leg in two adds c_leg and the extra ramp time."""
    straight = M.path_energy([(0, 0), (20, 0)])
    split = M.path_energy([(0, 0), (10, 0), (20, 0)])
    extra = M.c_leg_j + M.p_hover_w * (2 * M.leg_time(10) - M.leg_time(20))
    assert split - straight == pytest.approx(extra)


def test_degenerate_paths_cost_nothing():
    """A single point or a zero-length leg costs no transit energy."""
    assert M.path_energy([(3, 4)]) == 0.0
    assert M.path_energy([(3, 4), (3, 4)]) == 0.0
