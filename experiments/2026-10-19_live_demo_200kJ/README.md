# Live demo on W1 under 200 kJ, with RViz and the Gazebo window, 19 Oct 2026

Second flight of the live session of `2026-10-19_live_demo`, same
setup (`sim_session.sh` with `open_display.sh` and
`open_sim_window.sh` from that directory, software rendering through
WSLg), same policy c1_c2, alpha 0.5, but a budget of 200 kJ instead of
20 kJ: with it the mission ends on full coverage, not on the budget.
git d873aa8, fresh PX4 and Gazebo world, 0 failsafes.

| | |
|---|---|
| coverage | 100.0 % of W1 at 108.9 s, end 'coverage' |
| I1 | 0 violations, minimum margin +143.41 kJ |
| energy | 37.1 kJ used, 162.9 kJ left |
| landing | on the take-off zone, 1.0 m from its point, LANDED at 134.8 s |
| confirmed zones at the end | take-off point, and zone 2, 3972 m2 |

With this budget C1 never binds; the flight shows the display, not the
contribution.

## Two observations kept for later

**Hover beyond dwell_s.** `../2026-10-19_big_area/decision_latency.py`
on this session: hover between two paths median 1.7 s, max 4.6 s, 4 of
9 intervals over the 2 s that C1 budgets, 3 s and 1.0 kJ in all. With
RViz and the Gazebo window sharing the CPU, the decision latency
exceeds dwell_s even on W1. Harmless here, with 143 kJ of margin; see
`2026-10-19_decision_latency`.

**Zone 2 flickers between CONFIRMED and CANDIDATE.** In `events.jsonl`
zone 2 is confirmed at 33.9 s, then demoted with reason "1 pass, 2
needed" and confirmed again five times before 88 s. Each demotion
leaves C1 resting on the take-off point alone for one cycle. Not
investigated yet: the pass count of a matched zone should not fall.

    cd ~/ros2_ws/droneAfter/experiments
    python3 analyse_mission.py 2026-10-19_live_demo_200kJ/missions/*
    python3 2026-10-19_big_area/decision_latency.py 2026-10-19_live_demo_200kJ
