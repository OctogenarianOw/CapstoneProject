# Energy-Feasible UAV Exploration over an Online Registry of Safe Landing Zones

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

- **Report**: `docs/report/report_draft_v3.docx` (latest draft, built
  by `docs/report/build_report.py`).
- **Videos**: [demonstration missions in RViz](https://youtu.be/ekzL6OfNzm8)
  (10 min 40 s, nine missions on the three worlds, each beside its
  energy chart) and [one W1 mission in Gazebo](https://youtu.be/npNtkhwtkUM)
  (1 min 10 s, camera following the aircraft). Filmed missions are
  slowed by rendering: they illustrate, they are not measurements.
- **Flight code of the results**: tag `thesis-2026-10-27`.

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
  margin left, both rescaled over the admissible set at each decision,
  with alpha as the study parameter.

The property C1 must keep is the invariant **I1**: whenever the
registry holds a confirmed zone, the energy left covers the return to
one of them and the landing without touching the reserve. The mission
checks it every second and logs every violation. The other invariants
(I2 to I5: no safe claim on unseen ground, no setpoint over a known
obstacle, two distinct passes before a zone is confirmed, no landing
outside the registry except a logged emergency) are listed in the
specification, section 3.4.

## Results

Final campaigns, 23 and 24 Oct 2026: 210 PX4 SITL missions, same code,
three worlds (W1 60 m at 16 kJ, W2 120 m at 48 kJ, W3 160 m at 80 kJ),
seven configurations, ten missions each. B0 explores the nearest
frontier with no energy reasoning, B1 the largest gain with a return at
30 % of the battery, B3 applies C1 and C2 with alpha in {0, 0.25, 0.5,
0.75, 1}.

| quantity | value |
|---|---|
| missions violating I1 | **C1: 0 of 150**; B0: 30 of 30 (every one ran out of energy in flight, a crash on a real battery); B1: 11 of 30 (10 on W1, where 30 % is too late) |
| energy C1 charged against energy measured, in flight | above the measurement in 2530 of 2535 frontier transits |
| energy model `E = P_hover t + k_move d + c_leg n_legs` | 285.64 W, -0.400 J/m, 65.2 J; transits within 0.16 %, unseen validation flights within 0.13 to 0.26 % |
| effect of alpha on coverage | interior optimum, alpha 0.5 to 0.75, above both ends on all three worlds (Holm-corrected p <= 0.0091) |
| effect of alpha on the confirmed landing area | rises with alpha (W3: 328 to 625 m2, Spearman +0.44) |
| loss of the zone in use, provoked during a return (W3) | detected 3.1 to 4.1 s after the obstacle appeared, I1 re-evaluated at that cycle, landing on another zone; reserve used at most 0.30 kJ |
| landing detection, reference map v6, 1.5 m footprint | recall 94.2 %, 0 dangerous false accepts |
| landing detection, last map of the 70 final W1 missions | 10 maps keep a dangerous landable cell at one raised edge; no mission landed on one |
| elevation map error on flat ground, v6 | RMS 0.240 m |
| landing precision, 15 landings | p95 radial error 0.180 m, room 0.364 m in the 1.5 m footprint |
| decision cycle | median 29, 103, 198 ms on W1, W2, W3; at most 664 ms |

Details, statistics and negative results: the report, Sections 6.2 to
6.8, and the README of each experiment directory.

## Status, 1 November 2026

All gates A to H of the specification (section 7) are passed: terrain
analysis, landing zone registry, energy model, frontiers, C1, C2,
autonomous mission, and the full evaluation (210 missions rather than
240, because the planned C1-only configuration B2 is B3 at alpha = 1 by
construction). Every figure is redrawn from the logs by a script. The
report draft v3 is written; what remains is its review, the
communication log and the generative AI reflection.

## Repository layout

```
docs/prompt-maitre-projet-uav.md   full specification (French): question, components,
                                   acceptance criteria, evaluation protocol, trap
                                   catalogue, operations; project state in section 10
docs/report/                       report drafts (.docx) and the script that builds them
src/uav_core/uav_core/             the package: pure modules (no rclpy) and ROS nodes
src/uav_core/test/                 120 tests, run without a simulator
src/uav_core/launch/               exploration.launch.py (nodes, RViz, alpha slider)
src/uav_interfaces/                messages: ElevationMap, EnergyState, executor
models/x500_slz/                   PX4 x500 with a downward 3D LiDAR
worlds/                            worlds W1 (validation), W2, W3 and their ground truth
px4_config/airframes/              airframe 4030_gz_x500_slz
tools/                             offline tools: world generation, map, terrain and
                                   landing zone validation against the truth
experiments/<date>_<topic>/        one directory per measurement: README, scripts,
                                   raw logs, MANIFEST.tsv of hashes where flown
data_survey_vN.npz                 saved elevation maps, v6 is the reference
```

Pure modules: `elevation_grid` (per-cell Kalman fusion), `terrain_analysis`,
`slz_detection`, `slz_registry`, `observation`, `frontier_detection`,
`planner`, `energy_model`, `mission_policy` (C1, C2, baselines),
`mission_fsm` (the decision state machine), `world_model`.
Nodes: `cloud_world` (LiDAR to map), `exploration_manager` (the only
component that decides), `trajectory_executor` (the only one that drives
PX4), `energy_monitor` (rotor shaft energy), plus the test missions
`survey_mission`, `square_mission`, `offboard_demo`. The nodes exchange
topics only; `experiments/2026-11-01_rqt_graph` shows the graph ROS
reports in flight.

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

`experiments/sim_session.sh` starts PX4 SITL on a fresh world with reset
PX4 parameters, the XRCE-DDS agent and the bridges, runs the given
nodes, and cleans everything up afterwards (`WORLD=w2` or `WORLD=w3`
for the other worlds):

```bash
cd ~/ros2_ws/droneAfter/experiments
bash sim_session.sh /tmp/mission 400 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=100.0" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=16000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.75 -p area_half_m:=30.0 -p log_root:=/tmp/mission/missions" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
python3 analyse_mission.py /tmp/mission/missions/*
```

Policies: `nearest` (B0), `gain` (B1, return at a fixed battery
threshold), `c1_gain` (C1 alone), `c1_c2` (B3). Every mission writes its
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
| `2026-10-19_registry_flicker` | the registry flicker | touchdown jumps over 5 m 5, 12 -> 0, 0 after the fix |
| `2026-10-19_landing_p4` | landing precision, E_land | p95 0.180 m; E_land exact alone, +3 % inside missions |
| `2026-10-19_campaign_w1` | gates E and F, 80 missions | C1 violations all caused by the registry flicker; alpha effect on margin and energy, withdrawn on 20 Oct |
| `2026-10-20_campaign_w1_fix` | gates E and F after the registry fix | C1 missions breaking I1 31/60 -> 0/60 |
| `2026-10-20_gate_f_16kJ` | gate F, 50 missions at 16 kJ | alpha moves cells per kJ, margin, energy left; coverage optimum at alpha 0.75 |
| `2026-10-20_footprint_p4` | footprint 1.2 against 1.5 m, offline | 1.2 m rejected, severe false accepts 0 -> 2 on v6, 14 -> 44 on mission maps; 1.5 m kept |
| `2026-10-20_w2_first` | W2, the world of rare landing zones | full coverage 97 kJ, 0 false accepts, three pads confirmed |
| `2026-10-21_campaign_w2` | gates E and F on W2, 70 missions at 48 kJ | C1 missions breaking I1 0/50 (B0 10/10, B1 1/10) |
| `2026-10-21_pit_calibration`, `_2` | can a pitted pad hide from afar (W3)? | 1 m pits never revealed, 2 and 3 m pits seen from 10 to 17 m: W3 not built on pits |
| `2026-10-21_p5_*`, `2026-10-22_p5_*`, `2026-10-23_p5_noninferiority` | P5, a one-cell map offset on the ramps | pose taken after the scene; position at the scan time minus 0.060 s, integrated 23 Oct: offset (-1, 0) -> (0, 0), ramps 0.34 -> 0.16 m RMS, dangerous false accepts 3-4 -> 0 |
| `2026-10-22_w3_design`, `_w3_first` | W3, the mixed world | town, road, forest, scrub, rocks and hills over 240 m |
| `2026-10-22_campaign_w3` | first W3 campaign, N = 5 | gate F reproduced; gate E failed, 2 of 25 C1 missions after the loss of their return target |
| `2026-10-22_i4_cells`, `_i4_cells_flights` | I4 counted per cell | W1 zone confirmed in 3 of 3 missions against 1, coverage 80.8 against 68.0 % |
| `2026-10-22_reselect_flights` | a target found unreachable ended the mission | blacklist and select again: 3 of 3 -> 0 of 3 |
| `2026-10-22_emergency_flights`, `_escape_flights` | emergency landing, escape margin | hold before an emergency landing, drift 20 -> 0.3 m; return on a 1 m escape margin |
| `2026-10-23_return_switch_flights`, `_switch_robust_flights` | switch a return to a cheaper zone | 0 I1 violation and reserve never used in 2 x 10 W3 missions |
| `2026-10-23_campaign_w3` | first final W3 campaign | stopped after 2 rounds: switches to zones lost within 2 s; led to the robust switch |
| `2026-10-23_campaign_w{1,2,3}_final` | gate H, the final evaluation, 210 missions | see Results |
| `2026-10-23_s95_depot` | zone loss provoked during a frontier transit | 0 trigger in 26 missions: a transit rarely relies on one zone |
| `2026-10-24_s95_return` | zone loss provoked during a return | 4 of 13 missions: reaction as designed, reserve covers at most 0.30 kJ |
| `2026-10-24_thesis_figures` | figures F1 to F14 and the report figures | each redrawn from the logs by its script; index in its README |
| `2026-10-31_report_analyses` | analyses for the report | C1 charge above the measurement in 2530 of 2535 transits; coverage plateau alpha 0.5 to 0.75 |
| `2026-10-31_video` | demonstration videos | filming W3 with RViz starved the simulation (25 to 60 % stale-pose scans); RViz at low priority and 5 Hz: 0.1 % at most; found the end-cell gap below |
| `2026-11-01_rqt_graph` | the architecture figure against the live ROS graph | every drawn topic found; the executor also reads the vehicle position, now drawn |

Every flown campaign keeps its raw logs with a `MANIFEST.tsv` of SHA-256
hashes, and its `setup.txt` records the git commit it flew.

## Known limits

Simulation only, without wind; the energy is the mechanical rotor
energy of the simulator, integrated on the system clock, so a mission
slowed by rendering is charged more. Static environment; one take-off
point per world, assumed free for the whole mission. One Gaussian per
map cell (P2): raised structures shrink at their edges, cells at the
foot of obstacles are falsely refused, and on mission maps a smoothed
raised edge can be accepted (10 of 70 final W1 maps). The landing
commit does not re-check that the end cell of a return is still
landable: found while filming, a correction is on branch
`return-end-check`, not merged. After C1 abandons a target, or after an
escape, the vehicle returns instead of selecting again. Open points and
their measurements are kept in the specification, section 10, and in
the report, Section 7.
