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
  NAVIGATING      fly it; every cycle check that the target can still be
                  reached, or select another (S12, every policy), and
                  re-check it under C1, the S9.5 reaction when the zone
                  it relied on is retired
  RETURNING       fly to the reachable confirmed zone of least energy;
                  replan if that zone is retired on the way, or if
                  another one becomes cheaper by more than a stop
  COMMIT_LANDING  descent commanded, no more re-evaluation (S15.2)
  LANDED          done
  EMERGENCY       no confirmed zone reachable, or budget depleted: hold
                  until at rest, then land where the aircraft is, logged
                  (the I5 exception)

A return turns to another confirmed zone when reaching it from where
the aircraft is costs less than reaching the current target by more
than one stop and restart (c_leg + P_hover v / a, 541 J at the
calibration), at most return_switch_max times per return. Measured
need (2026-10-22_campaign_w3): the return itself makes the second pass
over zones along its way, which get confirmed behind it, and a target
lost en route and confirmed again 3 s later was flown past, 30 m on to
a farther zone, with 1.63 kJ of the reserve used (B3a000_r5). On those
logs the rule would have fired in 10 returns of 9 missions, saving 0.6
to 5.7 kJ. The threshold is the switch's own cost, so no switch costs
more than it saves under the model; the cap bounds flapping between
zones whose confirmation flickers (17 of the 20 pass-count demotions
of that campaign came within 10 s of the confirmation). Rejected: a
minimum age of 10 s for the new zone, which filters that flicker but
would have turned too late in B3a000_r5 (zone 41 back at 230 s, the
aircraft over the far zone at 242 s). Both energies are planned the
same way from the same cell, so the conservative start ramp of a plan
from a moving aircraft cancels out of the comparison.

Ends (S15.4), each logged distinctly: 'coverage' (no frontier left),
'budget' (none admissible under C1, or the B1 threshold), 'commanded'.
In every case the aircraft lands on a registry zone, never where it is,
except in EMERGENCY.

Invariant I1 is evaluated every cycle from the vehicle position and
logged with its margin; a negative margin is a violation, counted and
never silent. Under a C1 policy it also triggers an immediate return.
At the arrival of a return, the step that commits, the need is the
landing alone: the aircraft lands where it is. Planned from the vehicle
cell, it added a leg whenever that cell had stopped being a landing
cell in the cycle's registry (they are recomputed every cycle, I4 per
cell): 411 J for one cell, never flown, charged at the commit in 5 of
the 35 missions of 2026-10-22_campaign_w3 and counted as a violation in
one (B3a075_r2, -377 J, landed 570 J above the reserve). Decision of
Dorian, 23 Oct. Kept: the start ramp a plan from a moving aircraft
charges, which covers in part the braking of a return that changes
direction, not planned by the model (170 J measured in that mission).

Hover charged by C1 per visit (overhead_s), measured, not assumed. For
each frontier path flown to arrival the machine measures
    U = publish of the next command - decision - planned flight time
the time C1 had to cover beyond the plan: the computation before the
path left, the settling, the wait and computation at the frontier. The
next decisions charge the largest U of the last latency_window visits
plus the largest rise between two consecutive ones, floored at dwell_s.
Why: with dwell_s alone, 2 s, the first 190 m flight of 19 Oct spent
301 s of hover C1 never counted, as its decision step grew to 21.5 s,
and broke I1 7 times. A window maximum alone lags a latency that grows
with the map (worst shortfall 3.58 kJ on that flight); adding the
recent rise bounds the growth too (1.64 kJ), and on a fast machine
stays at dwell_s. Rejected: a fixed larger dwell_s, which would charge
a slow machine's latency to every mission. No bound known at decision
time can cover an unbounded jump, so a visit whose U exceeds what was
charged is logged as an 'overrun' event: the assumption is measured in
every mission, and the reserve R covers the shortfall.

A frontier whose cells are still frontier after the visit is
blacklisted (S11): more than half of its cells unchanged at the first
cycle after arrival.

Every command stays pending until the executor state confirms it (not
idle after a take-off, echoing the path id, landing), and is resent
every command_retry_s otherwise. Measured need: in the budget demo of
19 Oct the manager had a valid pose before the executor, which refused
the take-off ("no pose yet"); sent once, the command was lost and the
aircraft never left the ground. The retry delay runs from the moment
the node published the command (sent()), not from the start of the
decision that produced it: in the first 190 m flight of 19 Oct
decisions took up to 21.5 s, every command looked overdue the moment
it left, and the next step resent it before the executor's status could
confirm it, 20 resends, one restarting a path in flight.
"""

from dataclasses import dataclass, field
import math

from uav_core.mission_policy import (escape_return, evaluate,
                                     invariant_margin, plan_return,
                                     PolicyParams, return_field)
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
    dwell_s: float = 2.0                 # floor of the hover per visit
    latency_window: int = 5              # visits behind overhead_s
    bootstrap_cycles: int = 2
    energy_stale_s: float = 2.0
    command_retry_s: float = 2.0
    # An emergency holds before it lands, as the return does since
    # 2026-10-20_landing_drift: LAND sent at 5 m/s carried the aircraft
    # 20 m on, into the W3 town (2026-10-22_reselect_flights). From
    # 5 m/s at 3 m/s2 the stop takes 1.7 s; settle_s leaves the position
    # loop time to come back to the hold point. Chosen. If the executor
    # never reports holding, it lands after hold_max_s anyway.
    emergency_settle_s: float = 3.0
    emergency_hold_max_s: float = 10.0
    # Turns to a cheaper zone per return, against flapping (header).
    return_switch_max: int = 3


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
    emergency_t: float = None   # when the emergency was declared
    held_since: float = None    # executor holding since, in an emergency
    emergency_landing: bool = False
    path_id: int = 0
    target: object = None       # Candidate being flown to
    target_zone: int = None     # zone id of a return
    check_visit: object = None  # frontier cells to check after arrival
    cycles_in_state: int = 0
    return_attempts: int = 0
    return_switches: int = 0    # turns to a cheaper zone, this return
    pending: object = None      # (Action, time sent), until confirmed
    violations: int = 0
    last_margin: float = None
    last_decision: object = None
    history: list = field(default_factory=list)
    overruns: list = field(default_factory=list)   # measured U, seconds
    decided: tuple = None       # (t, planned s, charged s) of the target
    visit: tuple = None         # decided, once arrived, until next command

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

    @staticmethod
    def _confirmed(action, inp):
        """Tell whether the executor state shows the action was taken."""
        if action.kind == 'takeoff':
            return inp.ex_state not in (None, EX_IDLE)
        if action.kind == 'land':
            return inp.ex_state in (EX_LANDING, EX_LANDED)
        if action.kind == 'path':
            return inp.ex_path_id == action.path_id
        return True

    def _retry(self, inp):
        """Resend the pending command if the executor ignored it."""
        if self.pending is None:
            return []
        action, sent = self.pending
        if self._confirmed(action, inp):
            self.pending = None
            return []
        if inp.t - sent < self.params.command_retry_s:
            return []
        self.pending = (action, inp.t)
        self._event(inp.t, 'retry', action=action.kind,
                    path_id=action.path_id)
        return [action]

    def step(self, inp):
        """Advance the mission by one step, return the actions."""
        actions = self._step(inp)
        if actions:
            self.pending = (actions[-1], inp.t)
            return actions
        return self._retry(inp)

    def sent(self, t):
        """
        Record that the node published the step's commands at time t.

        Restamps the pending command, and closes the measurement of the
        visit just completed: the first command after an arrival ends
        the time C1 had to cover beyond the planned flight.
        """
        if self.pending is not None:
            self.pending = (self.pending[0], t)
        if self.visit is not None:
            t_decision, planned, charged = self.visit
            u = t - t_decision - planned
            self.overruns.append(u)
            self._event(t, 'overrun' if u > charged else 'latency', u_s=u,
                        charged_s=charged, planned_s=planned,
                        next_charge_s=self.overhead_s())
            self.visit = None

    def overhead_s(self):
        """Hover C1 charges per visit, from the latency measured so far."""
        w = self.overruns[-self.params.latency_window:]
        if not w:
            return self.params.dwell_s
        rise = max([0.0] + [b - a for a, b in zip(w[:-1], w[1:])])
        return max(self.params.dwell_s, max(w) + rise)

    def _step(self, inp):
        """Advance the state machine, return new actions."""
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
            if self.emergency_landing:
                return []
            if inp.ex_state == EX_HOLDING and self.held_since is None:
                self.held_since = inp.t
            settled = (self.held_since is not None and inp.t - self.held_since
                       >= self.params.emergency_settle_s)
            if settled or (inp.t - self.emergency_t
                           >= self.params.emergency_hold_max_s):
                self.emergency_landing = True
                self._event(inp.t, 'emergency_land',
                            held_s=(None if self.held_since is None
                                    else inp.t - self.held_since))
                return [Action('land')]
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
                                              reserve, res, origin,
                                              self.overhead_s())
        if (margin is None or margin == -math.inf) and b is not None:
            # The nominal map has no way back from here: I1 holds if the
            # escape margin has one (escape_return), and the return
            # flies it.
            plan, zid = escape_return(w, self.registry.confirmed(), res,
                                      origin)
            if plan is not None:
                margin = b - reserve - (
                    self.model.path_energy(plan.waypoints_ne)
                    + self.model.hover_energy(self.overhead_s())
                    + self.model.e_land_j)
                zone_id = zid
                self._event(inp.t, 'escape_margin', margin_j=margin,
                            zone_id=zid, length_m=plan.length_m)
        if (self.state == RETURNING and b is not None and self._arrived(inp)
                and any(z.zone_id == self.target_zone
                        for z in self.registry.confirmed())):
            # Arrived over the target: this step commits and the aircraft
            # lands where it is (header), no leg to fly.
            margin = b - reserve - (self.model.hover_energy(self.overhead_s())
                                    + self.model.e_land_j)
            zone_id = self.target_zone
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
        charged = self.overhead_s()
        p = PolicyParams(policy=self.params.policy, alpha=self.params.alpha,
                         reserve_j=self._reserve(inp), overhead_s=charged)
        d = evaluate(w, rfield, self.model, inp.remaining_j, p,
                     inp.resolution, inp.origin_ne)
        self.last_decision = d
        if d.best is None:
            self.end_reason = ('coverage' if d.reason == 'no_frontier'
                               else 'budget')
            return self._return(inp, rfield, d.reason)
        self.target = d.best
        self._log_guarantee(inp, d.best, charged)
        self.decided = (inp.t,
                        self.model.path_time(d.best.plan_go.waypoints_ne),
                        charged)
        self._go(inp.t, NAVIGATING,
                 f'frontier at {d.best.frontier.viewpoint_ne}')
        return [self._path(d.best.plan_go.waypoints_ne)]

    def _log_guarantee(self, inp, best, overhead_s):
        """
        Log what the guarantee of a new target rests on (S9.5, S17).

        C1 admits a target on its best return zone. S9.5 asks what
        happens if that zone goes, so each decision whose zone is not
        the take-off point also logs the need through the best other
        confirmed zone, and the I1 margin left without it from where the
        aircraft is. Instrumentation, no decision reads it: one more
        return field per such decision, 74 ms median on the 240 m grid
        of W3. Read by the W3 depot trigger (experiments/w3_depot.py),
        and for F13.
        """
        zones = self.registry.confirmed()
        z = next((z for z in zones if z.zone_id == best.zone_id), None)
        if z is None or z.provisional:
            return
        w, res, origin = inp.world, inp.resolution, inp.origin_ne
        others = [o for o in zones if o.zone_id != z.zone_id]
        budget = inp.remaining_j - self._reserve(inp)
        hover = self.model.hover_energy(overhead_s)
        # No other zone, or none reachable: nothing is left without it
        need_without, i1_without = math.inf, -math.inf
        if others:
            rf = return_field(w.traversable, others, res, origin)
            back, _ = plan_return(w.traversable, rf,
                                  best.frontier.viewpoint_ij, res, origin)
            if back is not None:
                need_without = (best.e_go
                                + self.model.path_energy(back.waypoints_ne)
                                + hover + self.model.e_land_j)
            if w.vehicle_ij is not None:
                home, _ = plan_return(w.traversable, rf, w.vehicle_ij, res,
                                      origin)
                if home is not None:
                    i1_without = budget - (
                        self.model.path_energy(home.waypoints_ne) + hover
                        + self.model.e_land_j)
        self._event(inp.t, 'guarantee', zone_id=z.zone_id,
                    viewpoint_ne=best.frontier.viewpoint_ne,
                    need_j=best.e_total, need_without_j=need_without,
                    budget_j=budget, i1_without_j=i1_without,
                    rests_on_zone=bool(need_without > budget))

    def _navigate(self, inp, rfield, retired_confirmed):
        if self._arrived(inp) or self._gave_up(inp):
            f = self.target.frontier
            if self._arrived(inp):
                self.check_visit = (f.viewpoint_ne, f.cells)
                self.visit = self.decided
                self._go(inp.t, SELECTING, 'arrived')
            else:
                hold = self.blacklist.record_failure(f.viewpoint_ne, inp.t)
                self._event(inp.t, 'blacklist', viewpoint_ne=f.viewpoint_ne,
                            hold_s=hold, unchanged=None)
                self._go(inp.t, SELECTING, 'executor arrival timeout')
            return self._select(inp, rfield)
        w = inp.world
        go = plan_path(w.traversable, w.dist_m,
                       self.target.frontier.viewpoint_ij, inp.resolution,
                       inp.origin_ne)
        if go is None:
            # The target can no longer be reached on the current map: an
            # obstacle was mapped at the viewpoint or on the way since the
            # decision, such as a roof read above the cruise ceiling as the
            # aircraft closed in (W3). This is the replanning of S12, for
            # every policy since it is I3, not C1: blacklist the viewpoint
            # like an unreachable frontier and select again. Before, under
            # C1 only, it counted as an inadmissible target and ended the
            # mission: the first W3 missions stopped with 21 to 74 kJ left
            # (2026-10-22_i4_cells_flights), never on W1 or W2.
            f = self.target.frontier
            hold = self.blacklist.record_failure(f.viewpoint_ne, inp.t)
            self._event(inp.t, 'replan', reason='target unreachable',
                        viewpoint_ne=f.viewpoint_ne, hold_s=hold)
            self._go(inp.t, SELECTING, 'target unreachable')
            return self._select(inp, rfield)
        if self.params.policy in C1_POLICIES:
            ok, need = self._target_admissible(inp, rfield, go)
            if not ok:
                self.end_reason = 'invariant'
                why = ('zone retired, target no longer admissible'
                       if retired_confirmed else 'target no longer admissible')
                self._event(inp.t, 'abort', reason=why, need_j=need,
                            remaining_j=inp.remaining_j)
                return self._return(inp, rfield, why)
        return []

    def _target_admissible(self, inp, rfield, go):
        """Re-check C1 for the frontier being flown to, from here."""
        w, res, origin = inp.world, inp.resolution, inp.origin_ne
        vp = self.target.frontier.viewpoint_ij
        back, _ = plan_return(w.traversable, rfield, vp, res, origin)
        if back is None:
            return False, math.inf
        # One overhead at the frontier, one at the zone before the land
        # command, as evaluate() charges them.
        need = (self.model.path_energy(go.waypoints_ne)
                + self.model.path_energy(back.waypoints_ne)
                + 2.0 * self.model.hover_energy(self.overhead_s())
                + self.model.e_land_j)
        return need <= inp.remaining_j - self._reserve(inp), need

    def _return(self, inp, rfield, why):
        w = inp.world
        plan = zone_id = None
        if w.vehicle_ij is not None:
            plan, zone_id = plan_return(w.traversable, rfield, w.vehicle_ij,
                                        inp.resolution, inp.origin_ne)
        if plan is None:
            # No nominal way back: the way out on the escape margin, rather
            # than an emergency landing onto what closed in (a roof).
            plan, zone_id = escape_return(w, self.registry.confirmed(),
                                          inp.resolution, inp.origin_ne)
            if plan is None:
                return self._emergency(
                    inp, 'vehicle off the traversable map'
                    if w.vehicle_ij is None else 'no confirmed zone reachable')
            self._event(inp.t, 'escape', reason=why, zone_id=zone_id,
                        length_m=plan.length_m)
        self.target_zone = zone_id
        if self.state != RETURNING:
            self.return_switches = 0
            self._go(inp.t, RETURNING, why)
        self._event(inp.t, 'return', zone_id=zone_id, reason=why,
                    length_m=plan.length_m,
                    energy_j=self.model.path_energy(plan.waypoints_ne))
        # Even over a landable cell already, fly to its centre and commit
        # only on arrival, which the executor reports at rest (under
        # 0.5 m and 0.3 m/s). Committing at once sent LAND at up to
        # 4.8 m/s: 13 of 154 committed landings of the W1 runs drifted
        # 2 to 9.2 m past the cell, once into the rough zone
        # (experiments/2026-10-20_landing_drift).
        return [self._path(plan.waypoints_ne)]

    def _returning(self, inp, rfield, retired_confirmed):
        if self.target_zone in retired_confirmed:
            return self._return(inp, rfield,
                                f'zone {self.target_zone} retired en route')
        if self._arrived(inp):
            return self._commit(inp)
        cheaper = self._cheaper_return(inp, rfield)
        if cheaper:
            self.return_switches += 1
            return self._return(inp, rfield, cheaper)
        if self._gave_up(inp):
            self.return_attempts += 1
            if self.return_attempts > 2:
                self._event(inp.t, 'return_timeout',
                            attempts=self.return_attempts)
                return self._commit(inp)
            return self._return(inp, rfield, 'return arrival timeout')
        return []

    def switch_cost_j(self):
        """One stop and restart: two half ramps and a leg's fixed cost."""
        m = self.model
        return m.c_leg_j + m.p_hover_w * m.cruise_speed_ms / m.accel_ms2

    def _cheaper_return(self, inp, rfield):
        """Why to turn to another confirmed zone (header), or None."""
        w = inp.world
        if (w is None or w.vehicle_ij is None
                or self.return_switches >= self.params.return_switch_max):
            return None
        best, zone_id = plan_return(w.traversable, rfield, w.vehicle_ij,
                                    inp.resolution, inp.origin_ne)
        if best is None or zone_id == self.target_zone:
            return None
        current = [z for z in self.registry.confirmed()
                   if z.zone_id == self.target_zone]
        plan = None
        if current:
            plan, _ = plan_return(w.traversable,
                                  return_field(w.traversable, current,
                                               inp.resolution,
                                               inp.origin_ne),
                                  w.vehicle_ij, inp.resolution,
                                  inp.origin_ne)
        if plan is None:
            return (f'zone {zone_id}, zone {self.target_zone} no longer '
                    f'reachable')
        saving = (self.model.path_energy(plan.waypoints_ne)
                  - self.model.path_energy(best.waypoints_ne))
        if saving <= self.switch_cost_j():
            return None
        return (f'zone {zone_id} cheaper by {saving:.0f} J than zone '
                f'{self.target_zone}')

    def _commit(self, inp):
        self._go(inp.t, COMMIT_LANDING,
                 f'over zone {self.target_zone}, {self.end_reason}')
        return [Action('land')]

    def _emergency(self, inp, why):
        self._go(inp.t, EMERGENCY, why)
        self._event(inp.t, 'emergency_landing', reason=why,
                    exception='I5, landing outside the registry')
        # Hold first; the EMERGENCY state lands once at rest.
        self.emergency_t, self.held_since = inp.t, None
        return [Action('hold')]
