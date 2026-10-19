"""
Mission state machine of the exploration manager (spec S15), pure.

Role: decide, from the world model, the landing zone registry, the
energy left and the executor state, what the aircraft does next, and
log why. The node around it only moves messages; keeping the decisions
here makes every rule testable without a simulator.

States (S15.1):
  INIT            wait for a valid pose, the energy signal and an idle
                  executor, then command the take-off
  TAKEOFF         climb; at altitude the take-off point enters the
                  registry as a provisional zone (S15.3, policy a)
  BOOTSTRAP       a few map cycles so the first catalogue is meaningful
  SELECTING       catalogue -> C1 -> C2 (or a baseline), send the path
  NAVIGATING      fly it; every cycle re-check the target under C1, the
                  S9.5 reaction when the zone it relied on is retired
  RETURNING       fly to the reachable confirmed zone of least energy;
                  replan if that zone is retired on the way
  COMMIT_LANDING  descent commanded, no more re-evaluation (S15.2)
  LANDED          done
  EMERGENCY       no confirmed zone reachable, or budget depleted: land
                  where the aircraft is, logged (the I5 exception)

Ends (S15.4), each logged distinctly: 'coverage' (no frontier left),
'budget' (none admissible under C1, or the B1 threshold), 'commanded'.
In every case the aircraft lands on a registry zone, never where it is,
except in EMERGENCY.

Invariant I1 is evaluated every cycle from the vehicle position and
logged with its margin; a negative margin is a violation, counted and
never silent. Under a C1 policy it also triggers an immediate return.

A frontier whose cells are still frontier after the visit is
blacklisted (S11): more than half of its cells unchanged at the first
cycle after arrival.
"""

from dataclasses import dataclass, field
import math

from uav_core.mission_policy import (evaluate, invariant_margin,
                                     plan_return, PolicyParams,
                                     return_field)
from uav_core.planner import plan_path

INIT, TAKEOFF, BOOTSTRAP = 'INIT', 'TAKEOFF', 'BOOTSTRAP'
SELECTING, NAVIGATING, RETURNING = 'SELECTING', 'NAVIGATING', 'RETURNING'
COMMIT_LANDING, LANDED, EMERGENCY = 'COMMIT_LANDING', 'LANDED', 'EMERGENCY'
C1_POLICIES = ('c1_gain', 'c1_c2')

# Executor states, as in uav_interfaces/ExecutorStatus
EX_IDLE, EX_TAKING_OFF, EX_HOLDING = 0, 1, 2
EX_FOLLOWING, EX_LANDING, EX_LANDED = 3, 4, 5


@dataclass
class MissionParams:
    """Mission parameters; alpha may change between cycles (S14)."""

    altitude_m: float = 8.0
    policy: str = 'c1_c2'
    alpha: float = 0.5
    reserve_fraction: float = 0.10       # R, fraction of the capacity
    return_threshold_fraction: float = 0.30   # B1 baseline only
    dwell_s: float = 2.0
    bootstrap_cycles: int = 2
    energy_stale_s: float = 2.0


@dataclass
class Action:
    """What the node must send to the executor."""

    kind: str                 # 'takeoff', 'path', 'land', 'hold'
    waypoints_ne: list = None
    path_id: int = 0


@dataclass
class Inputs:
    """One step's view of the world."""

    t: float
    vehicle_ne: tuple = None
    pose_valid: bool = False
    ex_state: int = None
    ex_path_id: int = 0
    ex_arrived: bool = False
    remaining_j: float = None
    capacity_j: float = None
    energy_age_s: float = math.inf
    world: object = None       # WorldModel of a new map, or None
    resolution: float = 1.0
    origin_ne: tuple = (0.0, 0.0)
    stop: bool = False


@dataclass
class MissionFSM:
    """The decision maker. step() returns the actions of one step."""

    params: MissionParams
    model: object               # energy_model.EnergyModel
    registry: object            # slz_registry.SlzRegistry
    blacklist: object           # frontier_detection.Blacklist
    log: object = None          # mission_log.MissionLog, optional
    state: str = INIT
    end_reason: str = ''
    path_id: int = 0
    target: object = None       # Candidate being flown to
    target_zone: int = None     # zone id of a return
    check_visit: object = None  # frontier cells to check after arrival
    cycles_in_state: int = 0
    return_attempts: int = 0
    violations: int = 0
    last_margin: float = None
    last_decision: object = None
    history: list = field(default_factory=list)

    # ------------------------------------------------------------ helpers
    def _event(self, t, kind, **data):
        if self.log is not None:
            self.log.event(t, kind, **data)

    def _go(self, t, state, why=''):
        self._event(t, 'state', old=self.state, new=state, why=why)
        self.history.append((t, self.state, state, why))
        self.state = state
        self.cycles_in_state = 0

    def _path(self, waypoints):
        self.path_id += 1
        return Action('path', waypoints_ne=list(waypoints),
                      path_id=self.path_id)

    def _reserve(self, inp):
        return self.params.reserve_fraction * (inp.capacity_j or 0.0)

    def _arrived(self, inp):
        return (inp.ex_state == EX_HOLDING and inp.ex_arrived
                and inp.ex_path_id == self.path_id)

    def _gave_up(self, inp):
        """Executor holding on our path without arrival: timed out."""
        return (inp.ex_state == EX_HOLDING and not inp.ex_arrived
                and inp.ex_path_id == self.path_id)

    # ------------------------------------------------------------ step
    def step(self, inp):
        """Advance the mission by one step, return the actions."""
        if self.state == INIT:
            ready = (inp.pose_valid and inp.ex_state == EX_IDLE
                     and inp.energy_age_s < self.params.energy_stale_s)
            if ready:
                self._go(inp.t, TAKEOFF, 'pose, energy and executor ready')
                return [Action('takeoff')]
            return []
        if self.state == TAKEOFF:
            if inp.ex_state == EX_HOLDING:
                z = self.registry.add_provisional(inp.vehicle_ne, inp.t)
                self._event(inp.t, 'zone', zone_id=z.zone_id, old='',
                            new='CONFIRMED', reason=z.reason)
                self._go(inp.t, BOOTSTRAP, 'at altitude')
            return []
        if self.state == COMMIT_LANDING:
            if inp.ex_state == EX_LANDED:
                self._go(inp.t, LANDED, self.end_reason)
            return []
        if self.state in (LANDED,):
            return []
        if self.state == EMERGENCY:
            if inp.ex_state == EX_LANDED:
                self._go(inp.t, LANDED, 'emergency landing')
            return []
        if inp.world is None:
            return []
        return self._cycle(inp)

    def _cycle(self, inp):
        """One map cycle: registry, invariant, then the state's rule."""
        w, res, origin = inp.world, inp.resolution, inp.origin_ne
        self.cycles_in_state += 1
        events = self.registry.update(w.landing, w.variance, w.count, res,
                                      origin, inp.vehicle_ne, inp.t)
        retired_confirmed = []
        for e in events:
            self._event(inp.t, 'zone', zone_id=e.zone_id, old=e.old,
                        new=e.new, reason=e.reason)
            if e.old == 'CONFIRMED' and e.new != 'CONFIRMED':
                retired_confirmed.append(e.zone_id)
        rfield = return_field(w.traversable, self.registry.confirmed(), res,
                              origin)
        b = inp.remaining_j
        reserve = self._reserve(inp)
        margin, zone_id, _ = invariant_margin(w, rfield, self.model, b,
                                              reserve, res, origin)
        self.last_margin = margin
        violated = margin is not None and margin < 0
        if violated:
            self.violations += 1
        if violated or retired_confirmed:
            # Every margin goes to cycles.jsonl through the node; events
            # keep the violations and the S9.5 re-evaluations.
            self._event(inp.t, 'invariant', margin_j=margin,
                        zone_id=zone_id, remaining_j=b, reserve_j=reserve,
                        violated=violated, retired=retired_confirmed)

        if b is not None and b <= 0.0:
            return self._emergency(inp, 'budget depleted')
        if inp.energy_age_s > self.params.energy_stale_s:
            return self._return(inp, rfield, 'energy signal lost')
        if inp.stop and self.state not in (RETURNING,):
            self.end_reason = 'commanded'
            return self._return(inp, rfield, 'commanded stop')
        c1 = self.params.policy in C1_POLICIES
        if c1 and margin is not None and margin < 0 and \
                self.state != RETURNING:
            self.end_reason = 'invariant'
            return self._return(inp, rfield, 'I1 violated')
        threshold = self.params.return_threshold_fraction * (
            inp.capacity_j or 0.0)
        if (self.params.policy == 'gain' and self.state != RETURNING
                and b is not None and b < threshold):
            self.end_reason = 'budget'
            return self._return(inp, rfield, 'B1 battery threshold')

        if self.state == BOOTSTRAP:
            if self.cycles_in_state >= self.params.bootstrap_cycles:
                self._go(inp.t, SELECTING, 'first maps in')
                return self._select(inp, rfield)
            return []
        if self.state == SELECTING:
            return self._select(inp, rfield)
        if self.state == NAVIGATING:
            return self._navigate(inp, rfield, retired_confirmed)
        if self.state == RETURNING:
            return self._returning(inp, rfield, retired_confirmed)
        return []

    # ------------------------------------------------------------ rules
    def _select(self, inp, rfield):
        w = inp.world
        if self.check_visit is not None:
            vp, cells = self.check_visit
            still = w.catalogue.frontier_mask[cells[:, 0], cells[:, 1]]
            if still.mean() > 0.5:
                hold = self.blacklist.record_failure(vp, inp.t)
                self._event(inp.t, 'blacklist', viewpoint_ne=vp,
                            hold_s=hold, unchanged=float(still.mean()))
            self.check_visit = None
        p = PolicyParams(policy=self.params.policy, alpha=self.params.alpha,
                         reserve_j=self._reserve(inp),
                         dwell_s=self.params.dwell_s)
        d = evaluate(w, rfield, self.model, inp.remaining_j, p,
                     inp.resolution, inp.origin_ne)
        self.last_decision = d
        if d.best is None:
            self.end_reason = ('coverage' if d.reason == 'no_frontier'
                               else 'budget')
            return self._return(inp, rfield, d.reason)
        self.target = d.best
        self._go(inp.t, NAVIGATING,
                 f'frontier at {d.best.frontier.viewpoint_ne}')
        return [self._path(d.best.plan_go.waypoints_ne)]

    def _navigate(self, inp, rfield, retired_confirmed):
        if self._arrived(inp) or self._gave_up(inp):
            f = self.target.frontier
            if self._arrived(inp):
                self.check_visit = (f.viewpoint_ne, f.cells)
                self._go(inp.t, SELECTING, 'arrived')
            else:
                hold = self.blacklist.record_failure(f.viewpoint_ne, inp.t)
                self._event(inp.t, 'blacklist', viewpoint_ne=f.viewpoint_ne,
                            hold_s=hold, unchanged=None)
                self._go(inp.t, SELECTING, 'executor arrival timeout')
            return self._select(inp, rfield)
        if self.params.policy in C1_POLICIES:
            ok, need = self._target_admissible(inp, rfield)
            if not ok:
                self.end_reason = 'invariant'
                why = ('zone retired, target no longer admissible'
                       if retired_confirmed else 'target no longer admissible')
                self._event(inp.t, 'abort', reason=why, need_j=need,
                            remaining_j=inp.remaining_j)
                return self._return(inp, rfield, why)
        return []

    def _target_admissible(self, inp, rfield):
        """Re-check C1 for the frontier being flown to, from here."""
        w, res, origin = inp.world, inp.resolution, inp.origin_ne
        vp = self.target.frontier.viewpoint_ij
        go = plan_path(w.traversable, w.dist_m, vp, res, origin)
        back, _ = plan_return(w.traversable, rfield, vp, res, origin)
        if go is None or back is None:
            return False, math.inf
        need = (self.model.path_energy(go.waypoints_ne)
                + self.model.hover_energy(self.params.dwell_s)
                + self.model.path_energy(back.waypoints_ne)
                + self.model.e_land_j)
        return need <= inp.remaining_j - self._reserve(inp), need

    def _return(self, inp, rfield, why):
        w = inp.world
        if w.vehicle_ij is None:
            return self._emergency(inp, 'vehicle off the traversable map')
        plan, zone_id = plan_return(w.traversable, rfield, w.vehicle_ij,
                                    inp.resolution, inp.origin_ne)
        if plan is None:
            return self._emergency(inp, 'no confirmed zone reachable')
        self.target_zone = zone_id
        if self.state != RETURNING:
            self._go(inp.t, RETURNING, why)
        self._event(inp.t, 'return', zone_id=zone_id, reason=why,
                    length_m=plan.length_m,
                    energy_j=self.model.path_energy(plan.waypoints_ne))
        if len(plan.cells) < 2:
            return self._commit(inp)
        return [self._path(plan.waypoints_ne)]

    def _returning(self, inp, rfield, retired_confirmed):
        if self.target_zone in retired_confirmed:
            return self._return(inp, rfield,
                                f'zone {self.target_zone} retired en route')
        if self._arrived(inp):
            return self._commit(inp)
        if self._gave_up(inp):
            self.return_attempts += 1
            if self.return_attempts > 2:
                self._event(inp.t, 'return_timeout',
                            attempts=self.return_attempts)
                return self._commit(inp)
            return self._return(inp, rfield, 'return arrival timeout')
        return []

    def _commit(self, inp):
        self._go(inp.t, COMMIT_LANDING,
                 f'over zone {self.target_zone}, {self.end_reason}')
        return [Action('land')]

    def _emergency(self, inp, why):
        self._go(inp.t, EMERGENCY, why)
        self._event(inp.t, 'emergency_landing', reason=why,
                    exception='I5, landing outside the registry')
        return [Action('land')]
