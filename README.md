# Geometry-aware UAV exploration with a continuous guarantee of a safe landing

Master's thesis, University of Technology Sydney, Capstone A/B.
Dorian Veloso, supervised by Dr. Graeme Best. Defence: December 2026.

A simulated multirotor explores unknown terrain with a downward 3D
LiDAR, builds a 2.5D elevation map, detects geometrically safe landing
zones online, and keeps at every moment the energy it needs to reach
one of them and land.

> **Research question.** How can a UAV autonomously search an unknown
> environment for safe landing zones, while ensuring continuous
> reachability of a geometrically confirmed safe site whenever one has
> been identified?

Simulation only: ROS 2 Jazzy, PX4 SITL v1.17, Gazebo Harmonic 8.11,
WSL2 Ubuntu 24.04. One aircraft, GPS localisation (EKF2), no wind.

## Contributions

- **C1, energy feasibility invariant.** A frontier is admissible only if,
  once reached, the energy left still covers the return to the nearest
  confirmed safe zone and the landing, with a fixed reserve:
  `E_go + E_return + E_land <= B - R`. The zones come from a registry
  built online, which grows when zones are confirmed and shrinks when
  an observation invalidates one, unlike the static, predefined return
  base assumed by prior energy-constrained exploration work.
- **C2, energy-margin utility.** Among admissible frontiers,
  `U = alpha G + (1 - alpha) M`: information gain against the energy
  margin left, with alpha as the study parameter.

The property C1 must keep is the invariant **I1**: whenever the
registry holds a confirmed zone, the energy left covers the return to
one of them and the landing without touching the reserve. The mission
checks it every second and logs every violation. The other invariants
(I2 to I5: no safe claim on unseen ground, no setpoint over a known
obstacle, two distinct passes before a zone is confirmed, no landing
outside the registry except a logged emergency) are listed in the
specification, section 3.4.

## Status, 20 October 2026

| gate | component | status |
|---|---|---|
| A | terrain analysis (slope, roughness, step) | done, slope and roughness separated against the analytic truth |
| B | landing zone detection and dynamic registry | done; a flicker of large zones between CONFIRMED and CANDIDATE found in the campaign and fixed (c7fc2aa); in-flight invalidation scenario on a deceptive world (W3) to do |
| C | energy model | done, within 1.0 % of unfitted flights from the plan, 0.26 % with the per-leg term |
| D | frontier detection on the map | done |
| E | C1 | **passed** on 20 Oct: the same 80-mission W1 campaign with the registry fix alone, 0 of 60 C1 missions break I1 (31 of 60 before), against 10 of 10 B0 and 6 of 10 B1; no C1 mission touches its reserve |
| F | C2, effect of alpha | **passed** on 20 Oct (W1, 16 kJ, 50 missions): alpha moves cells per kJ (Spearman +0.79), the minimum margin and the energy left (p < 0.001), with 0 violations; coverage peaks at alpha 0.75, 81 % against 67 % with pure information gain (the B2 ranking, p = 0.017) |
| G | autonomous mission, take-off to landing on a confirmed zone | done, full coverage of W1 in 97 s |
| H | full evaluation, 3 worlds, 240 missions | to do |

Measured reference figures (details and conditions in each experiment):

| quantity | value |
|---|---|
| elevation map error on flat ground, reference map v6 | RMS 0.240 m |
| landing detection on v6, 1.5 m footprint | recall 94.2 %, 0 severe false accepts (one map) |
| landing detection on the last map of 80 missions, 1.5 m | 14 maps hold one severe false accept, at two cells of the rough zone where the map smooths 0.5 m steps (open) |
| energy model, P_hover / per-metre / per-leg | 285.64 W / -0.400 J/m / 65.2 J |
| online energy monitor against the offline instrument | 0.000 % over a 21.9 kJ flight |
| landing precision, 15 landings | p95 radial error 0.180 m, footprint 1.2 m by the rule of S16 (1.2 m measured offline and rejected: severe false accepts 0 -> 2 on v6, 14 -> 44 on 80 mission maps; 1.5 m kept) |
| C1 under 5 s of injected decision latency, before and after charging the measured latency | I1 violations 2, 2, 1 -> 0, 0, 0 |

## Repository layout

```
docs/prompt-maitre-projet-uav.md   full specification (French): question, components,
                                   acceptance criteria, evaluation protocol, trap
                                   catalogue, operations; state in sections 7 and 10
src/uav_core/uav_core/             the package: pure modules (no rclpy) and ROS nodes
src/uav_core/test/                 92 tests, run without a simulator
src/uav_core/launch/               exploration.launch.py (nodes, RViz, alpha slider)
src/uav_interfaces/                messages: ElevationMap, EnergyState, executor
models/x500_slz/                   PX4 x500 with a downward 3D LiDAR
worlds/                            validation world W1 and its analytic ground truth
px4_config/airframes/              airframe 4030_gz_x500_slz
tools/                             offline tools: world generation, map, terrain and
                                   landing zone validation against the truth
experiments/<date>_<topic>/        one directory per measurement campaign: README,
                                   scripts, raw logs, MANIFEST.tsv of hashes
data_survey_vN.npz                 saved elevation maps, v6 is the reference
```

Pure modules: `elevation_grid` (per-cell Kalman fusion), `terrain_analysis`,
`slz_detection`, `slz_registry`, `observation`, `frontier_detection`,
`planner`, `energy_model`, `mission_policy` (C1, C2, baselines),
`mission_fsm` (the decision state machine), `world_model`.
Nodes: `cloud_world` (LiDAR to map), `exploration_manager` (the only
component that decides), `trajectory_executor` (the only one that drives
PX4), `energy_monitor` (rotor shaft energy), plus the test missions
`survey_mission`, `square_mission`, `offboard_demo`.

## Build and test

PX4 needs the model, the world and the airframe linked or copied into
its tree, and `px4_msgs` cloned into `src/` (ignored by git): see the
specification, Annexe A.2.

```bash
cd ~/ros2_ws/droneAfter          # never colcon build from ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs uav_interfaces uav_core
source install/setup.bash
cd src/uav_core && python3 -m pytest test
```

## Run an autonomous mission

`experiments/sim_session.sh` starts PX4 SITL on a fresh validation
world with reset PX4 parameters, the XRCE-DDS agent and the bridges,
runs the given nodes, and cleans everything up afterwards:

```bash
cd ~/ros2_ws/droneAfter/experiments
bash sim_session.sh /tmp/mission 400 \
  map="ros2 run uav_core cloud_world" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=20000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.5 -p log_root:=/tmp/mission/missions" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
python3 analyse_mission.py /tmp/mission/missions/*
```

Policies: `nearest` (B0), `gain` (B1, return at a fixed battery
threshold), `c1_gain` (B2), `c1_c2` (B3). Every mission writes its
configuration, every decision cycle with all candidates, every event
and periodic map snapshots to `missions/<date>/`, so any figure can be
rebuilt from the log without the simulator. With PX4 and the agent
started by hand (Annexe A.3), `ros2 launch uav_core exploration.launch.py`
runs the same nodes with RViz.

## Experiments

| directory | question | result |
|---|---|---|
| `2026-10-17_energy_signal` | which energy signal does SITL offer? | PX4 battery is a ramp in armed time; rotor shaft power from Gazebo is the signal, hover 285.9 W |
| `2026-10-19_offboard_timestamp` | offboard failsafes of the test nodes | raw 1 Hz timesync stamps; extrapolated clock, failsafes 177 -> 0 |
| `2026-10-19_energy_calibration` | gate C | 16 flights, P_hover, k_move, c_leg, E_land; unfitted flights within 0.26 % |
| `2026-10-19_energy_monitor` | online monitor | matches the offline probe to 0.000 % |
| `2026-10-19_executor` | trajectory executor live | square energy predicted within 0.33 % |
| `2026-10-19_observation_thresholds` | UNKNOWN, SPARSE, OBSERVED thresholds | n_min and var_max set from the map error per observation count |
| `2026-10-19_observed_gate` | detection on OBSERVED cells only | v6 unchanged, 94.2 % recall, 0 severe |
| `2026-10-19_planner` | planner on v6 | 12 ms per distance field |
| `2026-10-19_policy_offline` | C1 and C2 offline | admissible frontiers 12 -> 1 as the budget falls 20 -> 6.5 kJ |
| `2026-10-19_registry_replay` | registry on the survey | passes counted over the whole track: 5 zones confirmed |
| `2026-10-19_live_map` | live map and frontiers | map at 1 Hz, 120 KB; no fusion below 2 m, recall 43.5 -> 94.6 % |
| `2026-10-19_first_autonomous` | gate G | full coverage of W1 in 97 s, landing on the take-off zone |
| `2026-10-19_budget_demo` | one mission per policy under 20 kJ | I1 violations: C1 0, B1 12, B0 25 |
| `2026-10-19_live_demo`, `_live_demo_200kJ` | display for a live demonstration | why flat ground was refused at 20 kJ; full coverage at 200 kJ |
| `2026-10-19_big_area` | C1 on a 190 m area | 7 violations from a decision latency C1 did not charge; 0 after the cycle fixes |
| `2026-10-19_decision_latency` | C1 under 5 s of injected latency | violations 2, 2, 1 -> 0, 0, 0 with the measured charge |
| `2026-10-19_campaign_w1` | gates E and F, 80 missions | C1 violations all caused by the registry flicker; alpha effect on margin and energy, withdrawn on 20 Oct |
| `2026-10-20_campaign_w1_fix` | gates E and F after the registry fix | C1 missions breaking I1 31/60 -> 0/60, gate E passed; alpha moves the time to the first zone only |
| `2026-10-20_gate_f_16kJ` | gate F, 50 missions at 16 kJ | alpha moves cells per kJ, margin, energy left; coverage optimum at alpha 0.75, 81 % against 67 % at pure gain |
| `2026-10-20_w2_first` | W2, the world of rare landing zones | full coverage 97 kJ, 0 false accepts, three pads confirmed; alpha changed live acts on 7 of 8 and 12 of 12 later choices |
| `2026-10-21_pit_calibration` | can a pitted pad hide from afar (W3)? 1 m pits | never revealed, even overhead: 0 of 72 within 5 m; one-cell negative obstacles are a blind spot |
| `2026-10-21_pit_calibration_2` | the same with 2 and 3 m pits and a hover | seen from 10 to 17 m already, never hidden: no deceptive pad by pits, W3 not built on them |
| `2026-10-21_campaign_w2` | gates E and F on W2, 70 missions at 48 kJ | C1 missions breaking I1 0/50 (B0 10/10, B1 1/10); W1 coverage optimum replicated, alpha 0.75 at 67 % against 61 and 60 % at the ends; figures F9 to F12 |
| `2026-10-19_registry_flicker` | the registry flicker | touchdown jumps over 5 m 5, 12 -> 0, 0 after the fix |
| `2026-10-19_landing_p4` | landing precision, E_land | p95 0.180 m; E_land exact alone, +3 % inside missions |
| `2026-10-20_footprint_p4` | footprint 1.2 against 1.5 m, offline | 1.2 m rejected, severe false accepts 0 -> 2 on v6, 14 -> 44 on mission maps; 1.5 m kept |

Every campaign keeps its raw logs with a `MANIFEST.tsv` of SHA-256
hashes, and its `setup.txt` records the git commit it flew.

## Known limits

Simulation only; no wind, so the energy model is optimistic against
real flight; a logical energy budget, not a realistic simulated
battery; static environment; one Gaussian per map cell, which shrinks
raised structures at their edges (P2, measured, not corrected); a
one-cell offset between map and truth on the ramps (P5, under
investigation). Open points and their measurements are kept in the
specification, section 10.
