# Thesis figures, index of spec section 6.2 (F1 to F14)

Every figure of the spec is regenerated from the repository by the
script next to it; none is drawn by hand. Figures F1 to F8 and F14 live
here; F9 to F12 are produced per campaign by that campaign's
`figures.py`, F13 by the S9.5 experiment. The final figures are those
of the final campaigns (23 and 24 Oct, same code on the three worlds),
not those of the earlier campaigns, kept for the record.

Run from this directory, ROS and the workspace sourced (F4 to F7 import
`uav_core`): `python3 fN_*.py`. Each script prints the numbers it draws.

| N° | content | file | numbers it shows |
|---|---|---|---|
| F1 | architecture, nodes and flows, contributions dashed | `f1_architecture.png` | 4 uav_core nodes, 2 bridges, real topic names; dashed: registry, C1, C2 |
| F2 | the four frames and the conversions | `f2_frames.png` | FLU to FRD (+0.15 m down), R(q) and position at the scan stamp minus 0.060 s, ENU to NED |
| F3 | validation world W1, geometry and truth zones | `f3_validation_world.png` | truth surface in 3D in place of a Gazebo render (choice in the header) |
| F4 | elevation map v6 against the truth, per-zone error | `f4_map_vs_truth.png` | flat RMS 0.240 m, ramp 3 deg 0.128 m, cube bias -1.667 m (P2) |
| F5 | slope distribution per zone, the 6 deg threshold | `f5_slope_noise.png` | flat p95 4.34 deg, 10 deg ramp p01 8.25 deg, 3 deg ramp p99 6.17 deg |
| F6 | slope and roughness kept apart | `f6_slope_roughness.png` | zone medians, map against truth |
| F7 | landing cells against the truth | `f7_zones_vs_truth.png` | recall 94.2 %, 46 false accepts, all benign, 0 severe |
| F8 | energy model calibration | `f8_energy_calibration.png` | P_hover 285.64 W, k_move -0.400 J/m, c_leg 65.2 J; transits within 0.16 %, validation -0.26 to -0.13 % |
| F9, F10 | chronogram of a C1 mission and of B1 | `../2026-10-23_campaign_w{1,2,3}_final/figures/f9_f10_chronogram.png` | energy, need and reserve per cycle; the B1 crossing |
| F11 | alpha sweep, coverage and minimum margin | `../2026-10-23_campaign_w{1,2,3}_final/figures/f11_metrics.png` | one dot per mission, N = 10 per configuration |
| F12 | trajectories, B1 against B3 | `../2026-10-23_campaign_w{1,2,3}_final/figures/f12_trajectories.png` | over each world's truth, landings and first violations |
| F13 | registry shrink at the return (S9.5) | `../2026-10-24_s95_return/figures/f13_B3a100_r{05,08,10,13}.png` | map before and after, reaction trajectory; margins -278, +11837, -461, -914 J |
| F14 | landing precision, footprint retained | `f14_landing_precision.png` | 15 landings, p95 0.180 m; room 0.364 m at 1.5 m |

`f5_flat_tail.py` (no figure) explains the tail of F5's flat ground,
p99 21.66 deg: 88 of 2484 flat cells read over 6 deg, 81 of them
within 3 m of another zone's edge, where the truth has a step and the
map smears it (P2); the truth is flat under all of them.

Worlds and budgets of F9 to F12: W1 at 16 kJ (the budget where alpha
acts on W1), W2 at 48 kJ, W3 at 80 kJ, reserve 10 %, seven
configurations, 210 missions.

Sources of the data: `data_survey_v6.npz` (F4 to F7, never
regenerated), `worlds/validation_truth.npz` (F3 to F7),
`../2026-10-19_energy_calibration/runs/results.json` (F8),
`../2026-10-19_landing_p4/summary.txt` (F14), the mission logs of each
campaign (F9 to F13).
