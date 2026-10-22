"""
Scripted scenarios for mission_fsm, without simulator.

The world is a flat observed disc around take-off (or the whole area for
full coverage), the registry and the blacklist are the real ones, and
the executor is played by the test through the Inputs it reports. Each
test is one rule of S15 or one safety case.
"""

from types import SimpleNamespace

import numpy as np
from uav_core.energy_model import CALIBRATION_2026_10_19 as MODEL
from uav_core.frontier_detection import Blacklist
from uav_core.mission_fsm import (BOOTSTRAP, COMMIT_LANDING, EMERGENCY,
                                  EX_FOLLOWING, EX_HOLDING, EX_IDLE,
                                  EX_LANDED, INIT,
                                  Inputs, LANDED, MissionFSM, MissionParams,
                                  NAVIGATING, RETURNING, SELECTING, TAKEOFF)
from uav_core.mission_policy import evaluate, PolicyParams, return_field
from uav_core.slz_registry import CONFIRMED, SlzRegistry, Zone
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


def test_ignored_takeoff_is_resent():
    """The executor stays idle (no pose yet): takeoff resent after 2 s."""
    m = fsm()
    assert [a.kind for a in m.step(inputs(0.0, EX_IDLE))] == ['takeoff']
    assert m.step(inputs(1.0, EX_IDLE)) == []
    assert [a.kind for a in m.step(inputs(2.5, EX_IDLE))] == ['takeoff']
    assert m.step(inputs(3.0, EX_HOLDING)) == []
    assert m.state == BOOTSTRAP


def test_unacknowledged_path_is_resent_with_its_id():
    """The executor never echoes path 1: the same path is resent."""
    m = fsm()
    first = airborne(m)[0]
    assert m.step(inputs(4.0, path_id=0)) == []
    again = m.step(inputs(5.5, path_id=0))
    assert len(again) == 1 and again[0].path_id == first.path_id
    assert again[0].waypoints_ne == first.waypoints_ne
    assert m.step(inputs(8.0, path_id=1)) == []


def test_slow_decision_is_not_resent_as_it_leaves():
    """A path published 10 s after its decision started waits 2 s."""
    m = fsm()
    first = airborne(m)[0]
    m.sent(13.0)
    assert m.step(inputs(13.1, path_id=0)) == []
    again = m.step(inputs(15.5, path_id=0))
    assert len(again) == 1 and again[0].path_id == first.path_id


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


def test_visit_latency_is_measured_from_decision_to_next_command():
    """U = next command published - decision - planned flight time."""
    m = fsm()
    airborne(m)
    planned = MODEL.path_time(m.target.plan_go.waypoints_ne)
    vp = m.target.frontier.viewpoint_ne
    t_arrival = 3.0 + planned + 10.0
    m.step(inputs(t_arrival, w=world(vp), vehicle=vp, path_id=1,
                  arrived=True))
    m.sent(t_arrival + 0.5)
    assert np.isclose(m.overruns[-1], 10.5)
    assert np.isclose(m.overhead_s(), 10.5)


def test_overhead_is_window_max_plus_rise_floored_at_dwell():
    """Largest of the last 5, plus the largest rise, at least dwell_s."""
    m = fsm()
    assert m.overhead_s() == 2.0
    m.overruns = [1.0, 4.0, 3.0]
    assert m.overhead_s() == 7.0
    m.overruns = [50.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    assert m.overhead_s() == 2.0


def test_measured_latency_is_charged_by_c1():
    """A target that fits with 2 s of hover does not with 60 s measured."""
    m = fsm()
    airborne(m)
    fits = m.target.e_total + 0.1 * CAP + 5000.0
    assert m.step(inputs(4, w=world(), remaining=fits)) == []
    assert m.state == NAVIGATING
    m.overruns = [60.0]
    m.step(inputs(5, w=world(), remaining=fits))
    assert m.state in (RETURNING, COMMIT_LANDING)
    assert m.end_reason == 'invariant'


def test_budget_too_small_returns_home_and_lands():
    """
    None admissible while I1 holds: end 'budget', stop over home, land.

    15 kJ left, 10 kJ reserve: the landing (3.2 kJ) fits, so I1 holds at
    home, but no round trip (6.5 kJ and more) fits in the 5 kJ left. The
    aircraft is over the home cell already: it still gets a path to the
    cell, and LAND only once the executor reports arrival, at rest
    (2026-10-20_landing_drift: LAND sent in flight drifted up to 9.2 m).
    """
    m = fsm()
    w = world()
    m.step(inputs(0, EX_IDLE))
    m.step(inputs(1))
    m.step(inputs(2, w=w, remaining=15000.0))
    actions = m.step(inputs(3, w=w, remaining=15000.0))
    assert m.last_margin > 0
    assert m.end_reason == 'budget'
    assert m.state == RETURNING and [a.kind for a in actions] == ['path']
    path = actions[0]
    assert m.step(inputs(4, w=w, remaining=15000.0, path_id=path.path_id,
                         ex_state=EX_FOLLOWING)) == []
    assert m.state == RETURNING
    actions = m.step(inputs(5, w=w, remaining=15000.0, path_id=path.path_id,
                            arrived=True))
    assert m.state == COMMIT_LANDING and [a.kind for a in actions] == ['land']
    m.step(inputs(6, ex_state=EX_LANDED))
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


def roofed(vp, vehicle=(0.0, 0.0)):
    """Return the flat disc with a 7 m roof over a viewpoint (I3: 6 m)."""
    ii, jj = np.indices((N, N))
    north, east = ORIGIN[0] + ii + 0.5, ORIGIN[1] + jj + 0.5
    disc = np.hypot(north, east) <= 18.0
    roof = (np.abs(north - vp[0]) <= 1.5) & (np.abs(east - vp[1]) <= 1.5)
    count = np.where(disc, 100, 0).astype(np.int32)
    var = np.where(disc, 4e-4, np.inf).astype(np.float32)
    elev = np.where(disc, np.where(roof, -7.0, 0.0), np.nan)
    return build_world(elev.astype(np.float32), var, count, RES, ORIGIN,
                       vehicle, WorldParams())


def test_unreachable_target_is_replaced_not_the_end():
    """
    S12: a target no path reaches any more is blacklisted and replaced.

    A roof read at 7 m over the viewpoint after the decision blocks it
    (I3). Under C1 and under the baseline B1 alike, the mission selects
    another frontier instead of returning: on the first W3 missions this
    case ended them with 21 to 74 kJ left.
    """
    for policy in ('c1_c2', 'gain'):
        m = fsm(policy=policy)
        airborne(m)
        vp = m.target.frontier.viewpoint_ne
        actions = m.step(inputs(4, w=roofed(vp), path_id=1))
        assert m.state == NAVIGATING and m.end_reason == ''
        assert [a.kind for a in actions] == ['path']
        assert m.target.frontier.viewpoint_ne != vp
        assert m.blacklist.blocked(vp, 5.0)


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


def test_guarantee_without_its_zone_is_logged():
    """
    S9.5: a decision on a remote zone logs the need without that zone.

    Home at the take-off point and a confirmed zone 10 m east; a target
    on the east side returns to that zone. Without it the return goes
    home: the logged need is larger, and the I1 margin without it is
    the margin of a return home from where the aircraft is.
    """
    events = []
    m = fsm()
    airborne(m)
    m.log = SimpleNamespace(
        event=lambda t, kind, **data: events.append((kind, data)))
    w = world()
    cells = np.array([(i, j) for i in range(29, 32) for j in range(39, 42)])
    m.registry.zones[99] = Zone(zone_id=99, state=CONFIRMED, cells=cells,
                                landing_cells=cells,
                                touchdown_ne=(0.5, 10.5))
    rfield = return_field(w.traversable, m.registry.confirmed(), RES, ORIGIN)
    p = PolicyParams(policy='c1_c2', alpha=0.5, reserve_j=0.1 * CAP,
                     overhead_s=2.0)
    d = evaluate(w, rfield, MODEL, CAP, p, RES, ORIGIN)
    east = [c for c in d.candidates if c.zone_id == 99 and c.admissible]
    m._log_guarantee(inputs(5, w=w), east[0], 2.0)
    (kind, g), = [e for e in events if e[0] == 'guarantee']
    assert g['zone_id'] == 99 and g['need_without_j'] > g['need_j']
    assert np.isfinite(g['i1_without_j']) and not g['rests_on_zone']
