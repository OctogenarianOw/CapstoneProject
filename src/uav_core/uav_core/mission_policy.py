"""
Frontier selection under the energy invariant: C1, C2 and baselines.

Role: given the frontier catalogue, the landing zone registry, the
energy model and the energy left, say which frontiers are admissible,
rank them, and measure the invariant I1 from where the vehicle stands.
Pure functions; the exploration manager calls them and holds the
authority (spec 3.3).

C1, energy feasibility invariant (S13): a frontier f is admissible iff
    E_go(vehicle -> f) + E_return(f -> slz*) + E_land <= B - R
  E_go     energy of the simplified path to the viewpoint, plus a hover
           of overhead_s for everything the plan does not contain: the
           computation before the path leaves, the settling, the wait
           and computation at the frontier before the next command (the
           t_overhead of S10). The mission state machine sets it from
           the decision latency it measures (mission_fsm.overhead_s);
           a constant here broke I1 on 19 Oct when the latency grew
  E_return energy of the simplified path from the viewpoint to the
           nearest CONFIRMED zone, the take-off point included (S15.3),
           plus a hover of overhead_s: the return too leaves one decision
           after the energy is read, and the land command one decision
           after the arrival. Without it, 5 s decisions took 1.5 kJ of
           a 2 kJ reserve at every landing of 19 Oct
           (experiments/2026-10-19_decision_latency, reserve_use.py)
  E_land   landing energy, the p95 of the landings the manager
           commanded (energy_model.PLANNING_2026_10_20), above the
           calibrated descent of the executor alone
  slz* is the zone nearest along the return field, seeded on every
  landable cell of every confirmed zone: one pass for all frontiers.
  Every candidate cell of a zone passed the footprint test of S8, so
  the aircraft may land on any of them; the return goes to the nearest
  one, not to the zone's interior touchdown point, which in a large
  flat zone can lie tens of metres away while the aircraft flies over
  landable ground (measured in experiments/2026-10-19_big_area). The
  take-off point, which has no cells, seeds its own point. With the
  per-leg term, the nearest cell by distance may differ from the
  cheapest by the cost of a leg or two (65 J each); accepted, as it
  needs one return field instead of one per zone.
  The return is planned on the current map, which is optimistic: the
  map at f is unknowable (S13). The reserve R covers it.

C2, energy margin utility (S14), among admissible frontiers only:
    share(f) = clip((B - R - E_total(f)) / (B - R), 0, 1)
    G(f) = gain(f), rescaled over the pool of this cycle to [0, 1]
    M(f) = share(f), rescaled over the pool of this cycle to [0, 1]
    U(f) = alpha G(f) + (1 - alpha) M(f)
  rescaled as (x - min) / (max - min) over the admissible frontiers,
  every candidate 1 when all are equal. Both terms then span [0, 1] at
  every decision, so alpha weighs the rank in information against the
  rank in margin among the frontiers on offer, with the same meaning
  from one cycle to the next (S14); share(f), the margin in budget
  terms, is logged as m_share.
  Until 20 Oct G was gain / max gain and M the share itself. Over the
  2225 decisions of the 60 C1 missions of 2026-10-20_campaign_w1_fix,
  G spread 0.88 between frontiers and the share 0.12 (medians), so G
  took over at alpha 0.10 (median) and 92 % of the decisions at alpha
  0.5 were already the pure-gain choice: alpha acted on [0, 0.2] only,
  and the campaigns saw alpha 0 apart from the rest. Rescaled, the
  choice leaves the pure-margin one at alpha 0.35 and joins the
  pure-gain one at 0.55 (medians), 52 % at alpha 0.5
  (experiments/2026-10-20_c2_balance). Rejected: rescaling M only,
  which leaves G at 0.88 against 1 and alpha 0.5 off balance (49 %,
  switch medians 0.40 and 0.65); a nonlinear map of alpha, which hides
  the imbalance in the knob. Cost: where every frontier leaves almost
  the same margin, the rescaling magnifies a small difference; C1 has
  already removed every frontier that is not safe, so that choice is
  between safe frontiers only. B1 and B2 rank by G alone, unchanged by
  a rescaling that keeps the order.

Policies (spec 5.2), one name each for the logs:
  nearest   B0, smallest path distance, no energy check
  gain      B1, largest G, no C1; the manager returns at a fixed
            battery threshold, the classical baseline
  c1_gain   B2, C1 then largest G
  c1_c2     B3, C1 then largest U(alpha)

Invariant I1 (3.4): when the registry holds a confirmed zone, the energy
left must cover the return from the current position and the landing,
reserve included. margin = B - R - (E_return(vehicle) + E_land), with
E_return as above, its decision latency included; a negative margin is
a violation, always reported, never silent. Before that term, I1 was
B - R - (path energy + E_land); cycles.jsonl logs overhead_s with the
margin, so the earlier definition can be recomputed from any log.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from uav_core.planner import (descend_plan, distance_field,
                              nearest_traversable, plan_path)

POLICIES = ('nearest', 'gain', 'c1_gain', 'c1_c2')


@dataclass
class PolicyParams:
    """Selection parameters, alpha changeable between cycles."""

    policy: str = 'c1_c2'
    alpha: float = 0.5
    reserve_j: float = 0.0
    overhead_s: float = 2.0     # hover charged per visit, see E_go


@dataclass
class Candidate:
    """One frontier with every number C1 and C2 looked at."""

    frontier: object
    plan_go: object = None
    plan_return: object = None
    zone_id: int = None
    e_go: float = math.inf
    e_return: float = math.inf
    e_total: float = math.inf
    admissible: bool = False
    g: float = 0.0
    m: float = 0.0
    m_share: float = 0.0        # margin as a share of the budget, before M
    u: float = -math.inf


@dataclass
class Decision:
    """Outcome of one selection cycle."""

    candidates: list = field(default_factory=list)
    best: Candidate = None
    reason: str = ''


@dataclass
class ReturnField:
    """Distance to the nearest confirmed zone, and who owns each source."""

    dist: np.ndarray
    source_zone: dict          # source cell -> zone id


def return_field(traversable, zones, resolution, origin_ne):
    """
    Seed the return field on the landing cells of the confirmed zones.

    The landing cells of a registry zone are those whose whole footprint
    was observed from two passes (I4, slz_registry); a zone given with
    cells only (tests, offline tools) seeds all of them. A zone without
    cells, the take-off point, seeds its touchdown point, moved onto the
    nearest free cell within 2 m if it falls in the inflation. Zone cells
    are flat by construction; those inside the inflation of a
    neighbouring obstacle are left out.
    """
    sources, owner = [], {}
    for z in zones:
        cells = getattr(z, 'landing_cells', None)
        if cells is None:
            cells = getattr(z, 'cells', None)
        if cells is not None and len(cells):
            free = traversable[cells[:, 0], cells[:, 1]]
            for i, j in cells[free]:
                cell = (int(i), int(j))
                sources.append(cell)
                owner[cell] = z.zone_id
            continue
        i = int((z.touchdown_ne[0] - origin_ne[0]) / resolution)
        j = int((z.touchdown_ne[1] - origin_ne[1]) / resolution)
        cell = nearest_traversable(traversable, (i, j),
                                   max_cells=int(math.ceil(2.0
                                                           / resolution)))
        if cell is not None:
            sources.append(cell)
            owner[cell] = z.zone_id
    if not sources:
        return ReturnField(np.full(traversable.shape, np.inf), {})
    return ReturnField(distance_field(traversable, sources) * resolution,
                       owner)


def plan_return(traversable, rfield, start, resolution, origin_ne):
    """Return path from a cell to the nearest zone, and that zone's id."""
    plan = descend_plan(traversable, rfield.dist, start, resolution,
                        origin_ne)
    if plan is None:
        return None, None
    return plan, rfield.source_zone.get(tuple(plan.cells[-1]))


def invariant_margin(world, rfield, model, b_current, reserve_j,
                     resolution, origin_ne, overhead_s=0.0):
    """I1 margin from the vehicle cell, the zone it relies on, the plan."""
    if world.vehicle_ij is None or not rfield.source_zone:
        return None, None, None
    plan, zone_id = plan_return(world.traversable, rfield, world.vehicle_ij,
                                resolution, origin_ne)
    if plan is None:
        return -math.inf, None, None
    need = (model.path_energy(plan.waypoints_ne)
            + model.hover_energy(overhead_s) + model.e_land_j)
    return b_current - reserve_j - need, zone_id, plan


def rescale(values):
    """(x - min) / (max - min) over a pool, every value 1 if all equal."""
    lo, hi = min(values), max(values)
    if hi - lo <= 1e-12:
        return [1.0 for _ in values]
    return [(v - lo) / (hi - lo) for v in values]


def evaluate(world, rfield, model, b_current, params, resolution,
             origin_ne):
    """Score every kept frontier and pick one, or say why none."""
    if params.policy not in POLICIES:
        raise ValueError(f'unknown policy {params.policy}')
    budget = b_current - params.reserve_j
    cands = []
    for f in world.catalogue.frontiers:
        c = Candidate(frontier=f)
        c.plan_go = plan_path(world.traversable, world.dist_m,
                              f.viewpoint_ij, resolution, origin_ne)
        if c.plan_go is not None:
            c.e_go = (model.path_energy(c.plan_go.waypoints_ne)
                      + model.hover_energy(params.overhead_s))
            c.plan_return, c.zone_id = plan_return(
                world.traversable, rfield, f.viewpoint_ij, resolution,
                origin_ne)
            if c.plan_return is not None:
                c.e_return = (model.path_energy(c.plan_return.waypoints_ne)
                              + model.hover_energy(params.overhead_s))
        c.e_total = c.e_go + c.e_return + model.e_land_j
        c.admissible = c.e_total <= budget
        cands.append(c)

    decision = Decision(candidates=cands)
    if not cands:
        decision.reason = 'no_frontier'
        return decision
    pool = cands if params.policy in ('nearest', 'gain') else [
        c for c in cands if c.admissible]
    if not pool:
        decision.reason = 'none_admissible'
        return decision
    for c in pool:
        c.m_share = (float(np.clip((budget - c.e_total) / budget, 0.0, 1.0))
                     if budget > 0 and math.isfinite(c.e_total) else 0.0)
    gs = rescale([c.frontier.gain for c in pool])
    ms = rescale([c.m_share for c in pool])
    for c, g, m in zip(pool, gs, ms):
        c.g, c.m = g, m
        c.u = params.alpha * c.g + (1.0 - params.alpha) * c.m
    if params.policy == 'nearest':
        best = min(pool, key=lambda c: c.frontier.path_dist_m)
    elif params.policy in ('gain', 'c1_gain'):
        best = max(pool, key=lambda c: (c.g, -c.frontier.path_dist_m))
    else:
        best = max(pool, key=lambda c: (c.u, -c.frontier.path_dist_m))
    decision.best = best
    decision.reason = 'selected'
    return decision
