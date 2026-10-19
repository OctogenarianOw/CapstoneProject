"""
Scripted scenarios for mission_fsm, without simulator.

The world is a flat observed disc around take-off (or the whole area for
full coverage), the registry and the blacklist are the real ones, and
the executor is played by the test through the Inputs it reports. Each
test is one rule of S15 or one safety case.
"""

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.frontier_detection import Blacklist
from uav_core.mission_fsm import (BOOTSTRAP, COMMIT_LANDING, EMERGENCY,
                                  EX_HOLDING, EX_IDLE, EX_LANDED, INIT,
                                  Inputs, LANDED, MissionFSM, MissionParams,
                                  NAVIGATING, RETURNING, SELECTING, TAKEOFF)
from uav_core.slz_registry import SlzRegistry
from uav_core.world_model import build_world, WorldParams

N, RES, ORIGIN = 60, 1.0, (-30.0, -30.0)
CAP = 100000.0


def world(vehicle=(0.0, 0.0), radius=18.0):
    """Return the world model of a flat observed disc."""
    ii, jj = np.indices((N, N))
    disc = np.hypot(ORIGIN[0] + ii + 0.5, ORIGIN[1] + jj + 0.5) <= radius
    count = np.where(disc, 100, 0).astype(np.int32)
    var = np.where(disc, 4e-4, np.inf).astype(np.float32)
    elev = np.where(disc, 0.0, np.nan).astype(np.float32)
    return build_world(elev, var, count, RES, ORIGIN, vehicle, WorldParams())


def fsm(**kw):
    """Return a mission with the real registry and blacklist."""
    return MissionFSM(MissionParams(**kw), MODEL, SlzRegistry(), Blacklist())


def inputs(t, ex_state=EX_HOLDING, w=None, remaining=CAP, vehicle=(0, 0),
           path_id=0, arrived=False, **kw):
    """Return one step of inputs."""
    return Inputs(t=t, vehicle_ne=vehicle, pose_valid=True, ex_state=ex_state,
                  ex_path_id=path_id, ex_arrived=arrived,
                  remaining_j=remaining, capacity_j=CAP, energy_age_s=0.1,
                  world=w, resolution=RES, origin_ne=ORIGIN, **kw)


def airborne(m, w=None):
    """Drive a mission from INIT to its first selection."""
    w = w or world()
    assert [a.kind for a in m.step(inputs(0, EX_IDLE))] == ['takeoff']
    assert m.state == TAKEOFF
    m.step(inputs(1, EX_HOLDING))
    assert m.state == BOOTSTRAP
    m.step(inputs(2, w=w))
    return m.step(inputs(3, w=w))


def test_takeoff_waits_for_energy_and_registers_home():
    """No take-off on a stale energy signal; home becomes a zone."""
    m = fsm()
    stale = inputs(0, EX_IDLE)
    stale.energy_age_s = 10.0
    assert m.step(stale) == [] and m.state == INIT
    airborne(m)
    home = m.registry.confirmed()[0]
    assert home.provisional and home.touchdown_ne == (0, 0)


def test_first_selection_sends_a_path_and_navigates():
    """After the bootstrap cycles, a frontier path is sent."""
    m = fsm()
    actions = airborne(m)
    assert m.state == NAVIGATING
    assert [a.kind for a in actions] == ['path'] and actions[0].path_id == 1
    assert m.target.admissible


def test_arrival_selects_the_next_frontier():
    """Arrival on our path id leads to a new selection and path."""
    m = fsm()
    airborne(m)
    vp = m.target.frontier.viewpoint_ne
    actions = m.step(inputs(4, w=world(vp), vehicle=vp, path_id=1,
                            arrived=True))
    assert m.state == NAVIGATING and actions[0].path_id == 2


def test_budget_too_small_returns_home_and_lands():
    """
    None admissible while I1 holds: end 'budget', commit, land.

    15 kJ left, 10 kJ reserve: the landing (3.2 kJ) fits, so I1 holds at
    home, but no round trip (6.5 kJ and more) fits in the 5 kJ left.
    """
    m = fsm()
    w = world()
    m.step(inputs(0, EX_IDLE))
    m.step(inputs(1))
    m.step(inputs(2, w=w, remaining=15000.0))
    actions = m.step(inputs(3, w=w, remaining=15000.0))
    assert m.last_margin > 0
    assert m.end_reason == 'budget'
    assert m.state == COMMIT_LANDING and [a.kind for a in actions] == ['land']
    m.step(inputs(4, ex_state=EX_LANDED))
    assert m.state == LANDED


def test_full_coverage_ends_with_a_return_to_the_zone():
    """No frontier left: end 'coverage', path home, land on arrival."""
    m = fsm()
    full = world(vehicle=(12.0, 0.0), radius=100.0)
    m.step(inputs(0, EX_IDLE))
    m.step(inputs(1))
    m.step(inputs(2, w=full, vehicle=(12.0, 0.0)))
    actions = m.step(inputs(3, w=full, vehicle=(12.0, 0.0)))
    assert m.end_reason == 'coverage' and m.state == RETURNING
    assert actions[0].kind == 'path'
    assert actions[0].waypoints_ne[-1] == (0.5, 0.5)
    actions = m.step(inputs(4, w=full, vehicle=(0.5, 0.5),
                            path_id=actions[0].path_id, arrived=True))
    assert m.state == COMMIT_LANDING and actions[0].kind == 'land'


def test_target_no_longer_admissible_aborts_to_return():
    """S9.5 reaction: the budget falls under the target's need en route."""
    m = fsm()
    airborne(m)
    need = m.target.e_total + 0.1 * CAP
    actions = m.step(inputs(4, w=world(), remaining=need - 500.0))
    assert m.state in (RETURNING, COMMIT_LANDING)
    assert m.end_reason == 'invariant'
    assert actions


def test_depleted_budget_is_an_emergency_landing():
    """No energy left: land where the aircraft is, logged as I5."""
    m = fsm(policy='nearest')
    airborne(m)
    actions = m.step(inputs(4, w=world(), remaining=-1.0))
    assert m.state == EMERGENCY and [a.kind for a in actions] == ['land']


def test_executor_timeout_blacklists_the_frontier():
    """Holding on our path without arrival: blacklist and select again."""
    m = fsm()
    airborne(m)
    vp = m.target.frontier.viewpoint_ne
    m.step(inputs(4, w=world(), path_id=1, arrived=False))
    assert m.blacklist.blocked(vp, 5.0)
    assert m.state in (NAVIGATING, SELECTING)


def test_b1_returns_at_its_battery_threshold():
    """Baseline B1: return below 30 % of the capacity, end 'budget'."""
    m = fsm(policy='gain')
    airborne(m)
    m.step(inputs(4, w=world(), remaining=0.25 * CAP))
    assert m.state in (RETURNING, COMMIT_LANDING)
    assert m.end_reason == 'budget'
