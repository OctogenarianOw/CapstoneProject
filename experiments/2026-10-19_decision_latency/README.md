# C1 under a known decision latency, W1, 19 Oct 2026

Question: does C1 keep I1 when the manager's decisions are slow? The
first 190 m flight (`2026-10-19_big_area`) says no: with C1 charging a
constant 2 s of hover per frontier visit (`dwell_s`), its decision step
grew to 21.5 s, the aircraft spent 301 s of hover C1 never counted, and
I1 was violated 7 times. That flight cannot be repeated cheaply (12
minutes, and the performance fixes made it fast), so the latency is
injected on purpose here: `exploration_manager` parameter
`inject_delay_s` (7533c0d) holds every decision 5 s after its inputs are
read, as a slower computer would.

Setup, the same for every flight: W1, policy c1_c2, alpha 0.5, budget
20 kJ (about 60 % of a full coverage), reserve 10 % (2 kJ),
`inject_delay_s` 5, fresh PX4 and Gazebo, PX4 parameters reset
(87a7035), 0 failsafes. `bash run.sh LABEL 5.0 3`.

## Result

| flights | git | I1 violations | min I1 margin | coverage | energy left | reserve used |
|---|---|---|---|---|---|---|
| `before_d5_r1..3` | 87a7035, dwell_s constant | **2, 2, 1** | **-1.17, -1.19, -1.15 kJ** | 66.6, 66.3, 67.1 % | -0.83, -0.84, -0.87 kJ | 2.83, 2.84, 2.87 kJ |
| `after_d5_r1..3` | a4ac769, latency measured | 0, 0, 0 | +0.19, +0.24, +0.26 kJ | 56.0, 56.1, 56.1 % | +0.50, +0.55, +0.54 kJ | 1.50, 1.45, 1.46 kJ |

The three flights of each group repeat to within 0.07 kJ: the injected
latency dominates the run to run variation.

**Before.** First decision at 20.9 s, frontier at (11.5, -12.5); the
path leaves at 25.9 s. Next decision at 36.0 s: frontier at (11.5,
12.5) admitted with 2.12 kJ of C1 slack (11.39 kJ left, E_total
7.27 kJ, reserve 2 kJ). Its path leaves 5 s later; at 46.0 s the C1
re-check of the target fails, the manager aborts, its return path
leaves at 51.1 s; at 56.1 s I1 is at -1.17 kJ. The first visit had
taken 15.1 s beyond its planned flight for 2 s charged.

**After.** The state machine measures, for every visit flown to
arrival, U = next command published - decision - planned flight, and
charges the next decisions the largest U of the last 5 visits plus the
largest rise between two consecutive ones, at least `dwell_s`. In each
flight the first visit closes at 41.0 s with U = 14.8 to 15.1 s, logged
as an `overrun` event (charged 2 s). The second decision, at 36.0 s,
was still taken with 2 s, since its own computation ends the first
visit's measurement; but the C1 re-check of the next cycle, at 41.0 s,
charges the 15 s measured, finds the target no longer admissible and
returns. I1 stays positive throughout, at least +0.19 to +0.26 kJ. C1
costs coverage here: 56 % against 66 to 67 %.

## The predictor, chosen on the logs before these flights

`overrun_series.py` replays the visits of a mission and compares
predictors of U known at each decision. Worst shortfall of the charge,
as energy at P_hover:

| session | dwell_s alone | max of last 5 | max of last 5 + rise (kept) |
|---|---|---|---|
| first 190 m flight, latency growing to 57 s per visit | 15.76 kJ | 3.58 kJ | 1.64 kJ |
| 190 m re-flight, fast | 0.34 kJ | 0.23 kJ | 0.15 kJ |
| W1 live demo with RViz and Gazebo window | 0.91 kJ | 0.71 kJ | 0.22 kJ |
| W1 budget demo, headless | 0 | 0 | 0, charge 2.0 s throughout |

A window maximum lags a latency that grows with the map; the rise term
bounds that growth. On a fast machine the charge stays at `dwell_s`, so
C1 costs nothing there. Rejected: a fixed larger `dwell_s`, which would
charge every mission the latency of the slowest machine.

## What the reserve still pays for

`reserve_use.py` splits the end of every mission logged so far:

| session | reserve used | return decision to path | commit to land command | landing, measured | E_land, model |
|---|---|---|---|---|---|
| before_d5, 3 flights | 2.83 to 2.87 kJ | 5.0 s | 5.0 s | 3.46 to 3.49 kJ | 3.20 kJ |
| after_d5, 3 flights | 1.45 to 1.50 kJ | 5.0 s | 5.0 s | 3.47 kJ | 3.20 kJ |
| budget demo c1_c2 | 0 | 0.1 s | 0.1 s | 3.33 kJ | 3.20 kJ |
| live demo, 20 kJ | 0.49 kJ | 0.2 s | 0.4 s | 3.69 kJ | 3.20 kJ |
| 190 m re-flight | 0 | 0.2 s | 0.2 s | 3.63 kJ | 3.20 kJ |

Two things I1 does not count, both measured, neither fixed here:

1. **The latency of the last two decisions.** The return path and the
   land command each leave one decision after the energy was read: 5 s
   of hover each here, 0.1 to 0.4 s on a fast machine. C1 charges the
   measured latency per frontier visit; I1 and the return do not yet.
2. **The landing inside a mission costs more than E_land.** 3.30 to
   4.14 kJ in the sessions of this table, against 3.20 kJ calibrated
   from 12 descents (sd 15 J), the wrong direction (spec 5.3).
   *Corrected the same evening* (`2026-10-19_landing_p4`): the first
   version said "every autonomous mission", but these sessions were
   mostly loaded ones (RViz and Gazebo window, slow decisions). Over
   the 70 committed landings of the headless W1 campaign the excess is
   +3.0 % mean, +5.6 % p95; loaded sessions +12.6 % mean; 15 landings
   flown alone by the executor match E_land within 0.2 %.

Without injected latency, every C1 mission flown since the cycle fixes
stayed within its reserve (at most 0.49 kJ of 2 kJ used, live demo);
the baselines, which have no such guarantee, used 2.19 kJ (B1) and
5.67 kJ (B0) in the budget demo. The two biases above are two separate
changes, each with its own measurement: I1 charging the measured latency, and
E_land recalibrated on executor landings (with the landing precision
experiment P4 of S16, which needs the same ten landings).

## I1 and the return charge the latency (4f7e635), 20 Oct

Change: E_return in C1 and the I1 need get one `overhead_s` of hover,
the decision between the arrival over the zone and the LAND command.
Same protocol, `inject_delay_s` 5, three flights before on the code of
the day (e029ebd, which carries the registry fixes c7fc2aa and d115a97,
so `after_d5` of 19 Oct is no longer the baseline) and three after:

| flights | git | reserve used | coverage | I1 violations logged |
|---|---|---|---|---|
| `base_d5_r1..3` | e029ebd | 1.30, 1.46, 1.47 kJ | 64.3, 65.0, 64.5 % | 0, 0, 0 |
| `i1lat_d5_r1..3` | 4f7e635 | 1.47, 1.44, 1.47 kJ | 62.7, 64.2, 61.8 % | 4, 4, 4 |

**The reserve used did not fall.** In every flight the return starts at
40.4 to 40.5 s with the same energy, both before and after. The energy
was committed at 35.4 s, by the second decision, charged `dwell_s`
(2 s) because the first visit's latency (U 14.5 to 15.1 s) is only
measured when that visit closes. The change cannot act earlier than the
measurement it uses. That is the first-visit gap noted below, and it is
what the reserve pays for in this protocol.

**What the change does: I1 tells the truth.** `i1_truth.py` takes, at
the cycle the manager turns to RETURNING, the I1 margin under both
definitions (computable on every flight, since `overhead_s` is logged
with it), and the margin that actually remained, energy left at the end
minus the reserve (`i1_truth.txt`):

| flights | I1 before 4f7e635 | I1 from 4f7e635 | same, E_land p95 | actual |
|---|---|---|---|---|
| `after_d5_r1..3` | +3.15 to +3.22 kJ | -1.06 to -1.16 kJ | -1.24 to -1.35 kJ | -1.45 to -1.50 kJ |
| `base_d5_r1..3` | +3.22 to +3.38 kJ | -0.88 to -1.10 kJ | -1.06 to -1.29 kJ | -1.30 to -1.47 kJ |
| `i1lat_d5_r1..3` | +3.23 to +3.26 kJ | -0.90 to -1.09 kJ | -1.08 to -1.28 kJ | -1.44 to -1.47 kJ |

Before, I1 said +3.2 kJ while the mission was about to eat 1.5 kJ of
its reserve: a violation that happened and was never reported, against
"always reported, never silent" (S13, 3.4). From 4f7e635 the sign is
right and the error is 0.3 to 0.6 kJ, optimistic; the p95 E_land
(PLANNING_2026_10_20) takes 0.19 kJ more of it. The 4 violations per
flight are the cycles of the return and of the commit, true ones.

Coverage is 61.8 to 64.2 % after, 64.3 to 65.0 % before, but the two
groups fly the same frontier sequences (`base_d5_r1` and `i1lat_d5_r1`
the same two frontiers): the change did not alter a choice here, and
the small difference is not attributed to it.

## Also observed

- `before_d5` flights show 0 path resends: the retry fix 6bdc128
  holds under 5 s decisions (20 resends in the first 190 m flight).
- The first visit of a mission is always charged `dwell_s`, nothing
  being measured yet. The flights of 20 Oct show it is what the reserve
  pays here: the 1.3 to 1.5 kJ used by every flight under 5 s of
  latency come from a decision taken before the latency was known. A
  seed from the bootstrap cycles is the candidate, as its own change.

## Failed sessions

`failed/px4_not_ready_r1..3`: the first attempt at the before flights,
19:20 to 19:32. PX4 never became ready ("Preflight Fail: heading
estimate invalid"), because of magnetometer offsets PX4 had learned
over the day's flights and saved; kept in `px4_params_drifted/`. Fixed
by resetting PX4 parameters before each session (87a7035). No ROS node
was started in these sessions.

## Files

    run.sh                 N flights of one label and injected delay
    before_d5_r*/          87a7035, after_d5_r*/ a4ac769,
    base_d5_r*/            e029ebd, i1lat_d5_r*/ 4f7e635
      setup.txt px4.log *.log missions/<date>/ analysis.txt latency.txt
    overrun_series.py      U per visit and predictors, from events.jsonl
    reserve_use.py         end of mission against the reserve
    i1_truth.py            I1 at the return decision against the outcome
    i1_truth.txt           its output
    px4_params_drifted/    PX4 parameter files of the stuck sessions
    failed/                the stuck sessions

    cd ~/ros2_ws/droneAfter/experiments
    python3 2026-10-19_decision_latency/overrun_series.py 2026-10-19_big_area/c1c2_200kJ_190m/missions/*
    python3 2026-10-19_decision_latency/reserve_use.py 2026-10-19_decision_latency/*_d5_r*
