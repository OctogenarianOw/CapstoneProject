"""
Unit tests for rotor_energy, synthetic and against the calibration data.

The integrator must give back, from the raw rotor commands of a
calibration flight, the segment energy the offline analysis of 19 Oct
computed with its own code (results.json).
"""

import json
from pathlib import Path

import numpy as np
import pytest
from uav_core.rotor_energy import EnergyIntegrator, K_F, shaft_power

CAL = (Path(__file__).resolve().parents[3] / 'experiments'
       / '2026-10-19_energy_calibration' / 'runs')


def test_hover_power_matches_the_force_balance():
    """At 806 rad/s on four rotors the shaft power is about 286.5 W."""
    w = np.sqrt(2.264 * 9.81 / (4 * K_F))
    assert w == pytest.approx(806.0, abs=0.5)
    assert shaft_power([w] * 4) == pytest.approx(286.5, abs=0.5)


def test_constant_power_integrates_to_power_times_time():
    """250 Hz samples of 300 W over 10 s give 3000 J."""
    i = EnergyIntegrator()
    for k in range(2501):
        i.add(k / 250.0, 300.0)
    assert i.consumed_j == pytest.approx(3000.0)
    assert i.samples == 2501 and i.gaps == 0


def test_gaps_are_counted_not_bridged():
    """A 2 s hole is not integrated across."""
    i = EnergyIntegrator(max_gap_s=0.5)
    i.add(0.0, 100.0)
    i.add(0.1, 100.0)
    i.add(2.1, 100.0)
    assert i.consumed_j == pytest.approx(10.0) and i.gaps == 1


@pytest.mark.skipif(not CAL.exists(), reason='calibration data not found')
def test_reproduces_the_offline_segment_energy():
    """Hover segment of cal_L10_r1 within 0.1 % of results.json."""
    run = CAL / 'cal_L10_r1'
    rotors = np.genfromtxt(run / 'rotors.csv', delimiter=',', names=True)
    rows = (line.split(',') for line in
            (run / 'phases.csv').read_text().splitlines()[1:])
    phases = {name: float(t) for t, name in rows}
    a, b = phases['HOVER_PRE'], phases['TRANSIT']
    i = EnergyIntegrator()
    for row in rotors:
        if a <= row['t_wall'] <= b:
            i.add(row['t_wall'], shaft_power([row[f'w{k}']
                                              for k in range(4)]))
    expected = json.loads((CAL / 'results.json').read_text())[
        'segments']['cal_L10_r1']['hover_pre']['E']
    assert i.consumed_j == pytest.approx(expected, rel=1e-3)
