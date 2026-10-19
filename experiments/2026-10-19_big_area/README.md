# A 190 x 190 m area under 200 kJ, before and after the cycle fixes, 19 Oct 2026

Question: on an area large enough for the battery, not the world, to
end the exploration, does C1 keep I1? W1 only covers 60 x 60 m and
costs 32.4 kJ to cover fully; here the manager explores a square of
half side 95 m (`area_half_m:=95.0`) in a 200 m map grid
(`grid_size_m:=200.0`), budget 200 kJ, reserve 10 % (20 kJ), policy
c1_c2, alpha 0.5, RViz on. Outside the 60 m of W1 the ground is the
flat plane of the validation world, so most of the area is landable.
Fresh PX4 and Gazebo for each flight, no reused world, failsafes
counted in `px4.log`.

## Results

| flight | git | I1 violations | min margin | coverage | energy left | end |
|---|---|---|---|---|---|---|
| `c1c2_200kJ_190m` | 39922ae | **7** | **-12.06 kJ** | 62.9 % | 2.8 kJ | I1 violated at 571.7 s, return, landed on the take-off zone 0.6 m from its point |
| `c1c2_200kJ_190m_fixed` | db7dc55 | 0 | +1.74 kJ | 95.6 % | 21.2 kJ | S9.5 abort at 583.1 s, return, landed on the take-off zone 0.4 m from its point |

The first flight is a failure of C1, not of a baseline: the aircraft
entered a state where the confirmed zone it relied on was out of reach
under the reserve, with C1 active.

## Cause of the violation, measured

`decision_latency.py` reads the trajectory executor's lines in
`ros.log` (one per path received, with the planned duration of its
profile) and measures the hover between two paths:

    hover after path k = receipt(k+1) - receipt(k) - planned(k)

C1 charges each visit with its planned flight plus `dwell_s` = 2 s of
hover at arrival.

| flight | hover between paths, median / max | hover beyond dwell_s | at P_hover | paths resent |
|---|---|---|---|---|
| first | 6.2 s / 37.6 s | 301 s in 29 of 33 intervals | 85.9 kJ | 20 |
| fixed | 1.1 s / 2.8 s | 2 s in 5 of 58 intervals | 0.4 kJ | 0 |
| `2026-10-19_live_demo_200kJ` (W1) | 1.7 s / 4.6 s | 3 s in 4 of 9 intervals | 1.0 kJ | 0 |

The hover grew with the map: 8.9 s per visit at 264 s, 23.7 s at
296 s, 30 to 36 s after 380 s. The decision step (registry update,
invariant, C1 over every candidate), measured in `events.jsonl` from
the state change of a decision to its path, grew from under 1 s before
80 s to 2.5 s at 128 s, 4.8 s at 286 s, then 14.7 to 21.5 s after
353 s.
Each decision costs hover twice: the computation at the departure
point, after the energy was read and before the path leaves, and the
wait plus computation at the next frontier, before the following path
leaves.

The last decision shows the mechanism. At 532.8 s, 46.63 kJ left, the
frontier at (-54.5, -21.5) was admitted with E_total 18.71 kJ, so a C1
slack of 7.93 kJ over the 20 kJ reserve, of which the flight there was
budgeted 2.04 kJ (4.9 s planned plus 2 s of dwell). The path left
18.7 s later, the aircraft arrived, then hovered through the next
cycle. At 571.3 s, 38.5 s after the decision, it had used 11.09 kJ
instead of 2.04: 9.05 kJ more than C1 counted, more than its 7.93 kJ of
slack, hence I1 at -1.12 kJ. The return then took another 17.5 s of
computation before its path left, at the same hover cost, so the
margin went on falling to -12.06 kJ on the way home.

The 20 resends are a second, smaller effect of the same latency: the
manager stamps a pending command with the time its decision started.
After a cycle longer than `command_retry_s` (2 s), the next 10 Hz step
sees an unconfirmed command (the executor's status is still queued
behind the long callback) and resends it at once. A resend restarts the
path from rest where the aircraft is: once (path 15) it happened 2.1 s
into the flight, with 10.0 of 16.5 m left.

The logged `timings_ms` hid all of this: they covered only the world
model (median 179 ms, max 423 ms in the first flight). Fixed in
13a2ba7, which times the decision and the display too.

## What changed between the two flights

Four commits, each measured offline on snapshots of the first flight
(`profile_cycle.py`, `measure_return_target.py`), no flight in between:

| commit | change | measured on the first flight's snapshots |
|---|---|---|
| 3c1fada | zone inscribed radii from one distance transform | cycle 9.45 -> 2.56 s at 500 s |
| 47d4adc | return to the nearest landable cell of a confirmed zone | I1 margin 10.9 -> 23.4 kJ at 533 s |
| 8466d20 | vectorised path simplification | cycle 2561 -> 853 ms at 500 s |
| db7dc55 | display layers built only when subscribed | display only |

The fixed flight is the measurement of the four together, so its
outcome is not attributable to one of them. What it shows: with the
decision latency back near one second, the same configuration keeps I1
over a 739 s mission. It also produced an S9.5 sequence on its own: at
583.1 s a confirmed zone was retired, the target was no longer
admissible, the manager aborted and returned with a +2.3 kJ margin.

## What is still open

C1 holds only while the decision latency stays under `dwell_s`, and
nothing checks it. Even in the fixed flight 5 visits overran it by up
to 0.8 s, and in the W1 live demo with RViz and the Gazebo window, 4 of
9 by up to 2.6 s. The guarantee therefore depends on the machine it
runs on, the native Linux demo machine included. Treated in
`2026-10-19_decision_latency`.

## Files

    c1c2_200kJ_190m/          first flight, git 39922ae
    c1c2_200kJ_190m_fixed/    re-flight, git db7dc55
      setup.txt               launch command, git HEAD and status
      px4.log ros.log ...     raw logs
      missions/<date>/        config.json, events.jsonl, cycles.jsonl, maps/
    decision_latency.py       hover between paths and resends, from ros.log
    profile_cycle.py          offline timing of on_map on snapshots
    measure_return_target.py  touchdown point versus nearest landable cell
    *.txt                     outputs quoted in the commits

    cd ~/ros2_ws/droneAfter/experiments
    python3 analyse_mission.py 2026-10-19_big_area/c1c2_200kJ_190m/missions/*
    python3 2026-10-19_big_area/decision_latency.py 2026-10-19_big_area/c1c2_200kJ_190m
