"""
Unit tests for mission_policy, on a synthetic take-off map.

The map is the situation after the climb: a flat observed disc of 18 m
around take-off, unknown beyond, frontier pieces all around. Checked:
C1 admits everything under a large budget and nothing under a budget
smaller than one round trip, the baselines ignore C1, alpha moves the
choice from margin to gain, the return goes to the nearest confirmed
zone, and the I1 margin is the budget left after return and landing.
"""

from types import SimpleNamespace

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.mission_policy import (evaluate, invariant_margin,
                                     PolicyParams, return_field)
from uav_core.world_model import build_world, WorldParams

N, RES, ORIGIN = 60, 1.0, (-30.0, -30.0)


def take_off_world(vehicle=(0.0, 0.0)):
    """Return the world model of a flat observed disc of 18 m."""
    ii, jj = np.indices((N, N))
    disc = np.hypot(ORIGIN[0] + ii + 0.5, ORIGIN[1] + jj + 0.5) <= 18.0
    count = np.where(disc, 100, 0).astype(np.int32)
    var = np.where(disc, 4e-4, np.inf).astype(np.float32)
    elev = np.where(disc, 0.0, np.nan).astype(np.float32)
    return build_world(elev, var, count, RES, ORIGIN, vehicle,
                       WorldParams())


def zone(zid, ne):
    """Return a confirmed zone stub with an id and a touchdown point."""
    return SimpleNamespace(zone_id=zid, touchdown_ne=ne)


HOME = [zone(1, (0.0, 0.0))]


def run(world, zones, b, **kw):
    """Evaluate the catalogue against a budget."""
    rf = return_field(world.traversable, zones, RES, ORIGIN)
    return evaluate(world, rf, MODEL, b, PolicyParams(**kw), RES, ORIGIN)


def test_large_budget_admits_every_frontier():
    """Every piece of the take-off circle is admissible with 1 MJ."""
    w = take_off_world()
    d = run(w, HOME, 1e6)
    assert len(d.candidates) >= 8 and all(c.admissible for c in d.candidates)
    c = d.best
    assert c.e_total == c.e_go + c.e_return + MODEL.e_land_j
    assert c.zone_id == 1 and d.reason == 'selected'


def test_budget_below_one_round_trip_admits_nothing():
    """C1 refuses all; the nearest baseline ignores it."""
    w = take_off_world()
    d = run(w, HOME, 6000.0)
    smallest = min(c.e_total for c in d.candidates)
    assert smallest > 6000.0
    assert d.best is None and d.reason == 'none_admissible'
    assert run(w, HOME, 6000.0, policy='nearest').best is not None


def test_reserve_counts_against_the_budget():
    """A budget that fits only without the reserve is refused with it."""
    w = take_off_world()
    smallest = min(c.e_total for c in run(w, HOME, 1e6).candidates)
    assert run(w, HOME, smallest + 1.0).best is not None
    assert run(w, HOME, smallest + 1.0, reserve_j=100.0).best is None


def test_alpha_moves_the_choice_from_margin_to_gain():
    """With alpha = 1 the largest gain wins, with 0 the largest margin."""
    w = take_off_world()
    by_gain = run(w, HOME, 2e4, alpha=1.0).best
    by_margin = run(w, HOME, 2e4, alpha=0.0).best
    d = run(w, HOME, 2e4, alpha=0.5)
    assert by_gain.frontier.gain == max(c.frontier.gain
                                        for c in d.candidates)
    assert by_margin.e_total == min(c.e_total for c in d.candidates)
    assert max(c.g for c in d.candidates) == 1.0


def test_return_goes_to_the_nearest_confirmed_zone():
    """With a second zone north, northern frontiers return there."""
    w = take_off_world()
    d = run(w, HOME + [zone(2, (12.0, 0.0))], 1e6)
    north = max(d.candidates, key=lambda c: c.frontier.viewpoint_ne[0])
    south = min(d.candidates, key=lambda c: c.frontier.viewpoint_ne[0])
    assert north.zone_id == 2 and south.zone_id == 1
    assert north.e_return < south.e_return


def test_return_goes_to_the_nearest_cell_of_a_large_zone():
    """
    A long zone whose interior point is far: return to its nearest cell.

    The zone covers north 2.5 to 12.5 m along east 8.5 m; its touchdown
    point, as the registry could place it, sits at its far end. A
    frontier viewpoint south of it returns to the near end of the zone,
    not to the touchdown point.
    """
    w = take_off_world()
    north = np.arange(32, 43)              # cells of north 2.5 to 12.5 m
    cells = np.stack([north, np.full_like(north, 38)], axis=1)
    big = SimpleNamespace(zone_id=3, touchdown_ne=(12.5, 8.5), cells=cells)
    d = run(w, [big], 1e6)
    south = min(d.candidates, key=lambda c: c.frontier.viewpoint_ne[0])
    end = south.plan_return.waypoints_ne[-1]
    assert south.zone_id == 3
    assert end == (2.5, 8.5)
    far = run(w, [zone(3, (12.5, 8.5))], 1e6)
    far_south = min(far.candidates,
                    key=lambda c: c.frontier.viewpoint_ne[0])
    assert south.e_return < far_south.e_return


def test_invariant_margin_at_home_is_budget_minus_landing():
    """Standing on the zone, I1 needs only the landing."""
    w = take_off_world()
    rf = return_field(w.traversable, HOME, RES, ORIGIN)
    margin, zid, _ = invariant_margin(w, rf, MODEL, 10000.0, 500.0, RES,
                                      ORIGIN)
    assert zid == 1
    assert margin == 10000.0 - 500.0 - MODEL.e_land_j


def test_invariant_margin_charges_the_decision_before_the_land_command():
    """On the zone, a 5 s latency adds 5 s of hover to the need of I1."""
    w = take_off_world()
    rf = return_field(w.traversable, HOME, RES, ORIGIN)
    margin, _, _ = invariant_margin(w, rf, MODEL, 10000.0, 500.0, RES,
                                    ORIGIN, overhead_s=5.0)
    assert np.isclose(margin, 10000.0 - 500.0 - MODEL.e_land_j
                      - MODEL.hover_energy(5.0))
