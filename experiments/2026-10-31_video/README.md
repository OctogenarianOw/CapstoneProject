# Filmed demonstration missions, 31 Oct 2026

The video linked from the report. Missions flown live in PX4 SITL and
Gazebo Harmonic, shown in RViz, each next to its own energy margin and
registry history drawn from its log. Rule followed: never a flight
alone, always the flight and the energy together, since the
contribution (C1, C2) is invisible in a picture of a flying vehicle.

**These are demonstrations, not measurements.** RViz renders in software
under WSLg and slows the simulation; the energy is integrated on the
system clock, so a filmed mission is charged more energy than the
headless missions of the final campaigns. No number of the report comes
from here. The missions are logged like any other (`runs/<name>/`), so
what the video shows can be checked.

## Pipeline

| step | file | what it does |
|---|---|---|
| views | `make_rviz.py` -> `rviz/<world>.rviz` | the demonstration RViz configuration, framed per world, 15 Hz |
| flights | `shoot.sh` | one fresh simulator per mission through `../sim_session.sh`, RViz opened by `open_display_video.sh`; the provoked loss of a zone reuses `../2026-10-24_s95_return/return_depot.py` |
| recording | `record.ps1` (Windows) | waits for an RViz window, records the desktop with ffmpeg (gdigrab; the window itself records black under WSLg), stops when the window closes; stores its start on the Windows clock and the WSL minus Windows offset |
| cut | `compose.py PLAN.json` | finds each mission's recording, crops the RViz view, draws the mission's chart revealed up to the current instant, stacks them, adds title cards, concatenates |

Synchronisation: each line of `mgr.log` carries the ROS wall time and
the mission time; with the offset stored by `record.ps1` the mission
time of every frame is known to a few hundred milliseconds.

Recordings and the cut video are large and stay outside the repository
(scratchpad); the scripts, the plans and the mission logs are here.
`plan.json` gives `uav_safe_landing_demo.mp4` (5 min 44 s, six missions),
`plan_final.json` adds the provoked loss and a closing card
(`uav_safe_landing_demo_full.mp4`).

## Missions flown, 31 Oct 18:07 to 19:14 (clock checked, 0 steps)

| run | world, policy | I1 violating cycles | min margin | energy left (reserve) | end |
|---|---|---|---|---|---|
| w3_b3a075 | W3, C1 + C2, alpha 0.75 | 0 | +2.36 kJ | 11.31 (8.0) kJ | commit on zone 58 |
| w1_b1 | W1, B1 (30 % return) | 22 | -1.93 kJ | 0.36 (1.6) kJ | commit, into the reserve |
| w1_b3a075 | W1, C1 + C2, alpha 0.75 | 0 | +1.27 kJ | 3.59 (1.6) kJ | commit |
| w2_b0 | W2, B0 (nearest) | 54 | -12.51 kJ | -4.17 (4.8) kJ | emergency landing |
| w2_b3a075 | W2, C1 + C2, alpha 0.75 | 0 | +0.87 kJ | 6.59 (4.8) kJ | commit |
| w3_b0 | W3, B0 (nearest) | 111 | -18.78 kJ | -6.49 (8.0) kJ | emergency landing |
| w3_depot_1..4 | W3, C1 + C2, alpha 1, crate trigger | 0 | +1.94 to +9.36 kJ | 12.8 to 19.5 (8.0) kJ | crates placed in run 4 only |
| w3_fix_depot_1..5 | same, with branch return-end-check | 0, 31, 0, 0, 27 | down to -2.49 kJ | 6.07 to 9.79 (8.0) kJ | crates placed in run 5 only |

No PX4 failsafe except one simulated-battery failsafe at the end of
w3_fix_depot_5.

**A defect found while filming (w3_depot_4).** The crates covered only
part of the target zone; the zone stayed CONFIRMED through its other
cells, so 'retired en route' never fired, the return flew on to its
original end, inside the field, and the arrival committed the landing
3.3 m from the field centre: the commit checks that the executor has
arrived, not that the cell is still landable. On the 150 C1 missions of
the final campaigns this happened once, harmlessly
(`../2026-10-31_report_analyses/commit_cell.py`, 149 of 150 commits over
a cell the map held landable). A first correction on branch
`return-end-check` (worktree ~/uav_commit, notes in
`docs/return-end-check.md` there) avoided the crates in run 5 but broke
I1 in runs 2 and 5 by replanning over flickering cells near the end of
returns; it is not merged.

## Second filming, 1 Nov 2026 (after Dorian's review)

Asked: the effect of alpha (same W2 mission at alpha 0.1 and 0.9, final
maps side by side), a B0 mission read as a crash when its energy reaches
zero, the W3 takes again if they can look better, and clearer cards.

**The 31 Oct W3 takes had a degraded map.** `map.log` counts the LiDAR
scans fused with the last received pose instead of the pose at the scan
instant (the P5 correction falls back to it when the pose stream lags):

| takes | stale-pose scans |
|---|---|
| final campaigns, 210 headless missions | 25 of 287 600 (W1 0, W2 8, W3 17) |
| 31 Oct, W1 and W2 filmed | 0 to 1.7 % |
| 31 Oct, every W3 filmed (11 takes) | 25 to 60 % (w3_b3a075: 812 of 1860) |

RViz renders in software on all 16 cores and starves Gazebo and PX4 on
the large W3 map. Effect on the picture, measured by replaying the
perception on every 10 s map snapshot (`green_to_red.py`): cells shown
green and red at the end, all flat in the truth, 624 of 2002 in
w3_b3a075 and 644 of 1412 in w3_b0, against 15 to 92 (B3a075_r01..r03)
and 155 to 212 (B0_r01..r02) in the headless W3 campaign, where they are
the declared P2-oblique false refusals. The report's numbers come from
the campaigns and are not affected.

Remedy, one change at a time (`open_display_light.sh`, `shoot_v2.sh`,
`shoot_v3.sh`; a take is kept at 1 % of stale scans or less):

| run | display | stale scans | kept |
|---|---|---|---|
| w3_b3a075_v2 | RViz nice 19, 4 render threads, 15 Hz | 16.86 % | no |
| w3_b3a075_v3 | same, 5 Hz (`rviz/w3_5hz.rviz`) | 0.09 % | yes, 57 green-to-red cells |
| w3_b0_v3 | same, 5 Hz | 0.00 % | yes |
| w2_a010, w2_a090 | nice 19, 15 Hz | 0.08 %, 0.00 % | yes |

5 Hz costs nothing on screen: the W3 segments are shown x4 and x5.

| run | world, policy | I1 violating cycles | min margin | energy left | seen | end |
|---|---|---|---|---|---|---|
| w3_b3a075_v3 | W3, C1 + C2, alpha 0.75 | 0 | +1.46 kJ | 10.28 kJ | 0.60 | commit on zone 4 |
| w3_b0_v3 | W3, B0 | 49 | -18.75 kJ | 0 at 273.1 s, -4.11 at landing | 0.48 | budget depleted in flight |
| w2_a010 | W2, C1 + C2, alpha 0.1 | 0 | +1.74 kJ | 7.33 kJ | 0.63 | commit on zone 2 |
| w2_a090 | W2, C1 + C2, alpha 0.9 | 0 | +3.53 kJ | 9.10 kJ | 0.65 | commit on zone 2 |

**B0 and the empty battery.** The budget is a counter in the manager,
not PX4's battery (spec S10): at zero the state machine declares
EMERGENCY, holds, and lands where it is. In the final campaigns all 30
B0 missions reached zero in flight and flew on 12.4 to 17.8 s before
touching down (-4.39 to -2.81 kJ). On a real battery each is a crash;
the cut (`compose_v2.py`) stops a B0 segment at that instant under a
crash banner.

`compose_v2.py plan_v2.json` gives `uav_safe_landing_demo_v2.mp4` and
`alpha_compare_w2.png` (the two final maps, drawn from the logs).

**World views (asked 1 Nov afternoon).** Each card that introduces a
simulation shows its world turning in Gazebo beside the text. `orbit/`:
`orbit.sh` opens the world in the Gazebo GUI (`orbit/gui.config`, no
PX4, server paused), `orbit_capture.py` moves the camera one step of a
circle at a time (CameraTracking, `/gui/move_to/pose`) and saves the
GUI's own render (Screenshot plugin, through the gz CLI). 300 renders a
turn, shown at 12.5 a second: a 24 s turn that loops. Radius and height:
W1 40 and 25 m, W2 155 and 98 m, W3 168 and 106 m. About one render a
second; one screenshot request in 64 to 248 went unanswered, now
retried. The renders were deleted after encoding (disk space); the
encoded turns are `work_v2/orbit_w1.mp4`, `_w2`, `_w3` next to the
recordings.

## The Gazebo film, 1 Nov 2026

A separate short video (`uav_gazebo_flight.mp4`, `compose_gazebo.py
plan_gazebo.json`): one W3 mission (C1 + C2, alpha 0.75, 80 kJ) seen in
the Gazebo GUI itself, the camera following the drone, with a corner
panel from the mission log (time, state, energy left against what C1
needs). `shoot_gz.sh` flies it with `open_gazebo_follow.sh` as display:
the GUI attaches to PX4's headless server, and a CameraTrack message
on /gui/track (FOLLOW_LOOK_AT, offset -5, -5, +3 m, gains 0.2 and 0.5)
keeps the camera on x500_slz_0; the default /gui/follow gain trails
the aircraft by tens of metres. The window is recorded from the desktop
(`record.ps1 -Title '*Gazebo*'`): Gazebo's VideoRecorder has no
service in this version (Harmonic 8.11), only its button.

| take | camera | window | stale scans | kept |
|---|---|---|---|---|
| w3_gazebo | 20 m away | maximised | 18.48 % | no, drone a few pixels wide |
| w3_gazebo_2 | 7.7 m | resized to 1296 x 806 by MoveWindow | 7.16 % | no, the 3D view stayed white after the resize |
| w3_gazebo_3 | 7.7 m | maximised | 18.39 % | yes, commit on zone 10 |

The kept take is over the 1 % gate of the RViz takes. The Gazebo picture
shows the simulated world, not the robot's map, so the stale poses do
not show; they make the mission's perception noisier than in the
measured missions, which the card states as "a demonstration, not a
measurement".

**On W1 as well** (`plan_gazebo_w1.json` -> `uav_gazebo_flight_w1.mp4`,
`bash shoot_gz.sh w1_gazebo validation 100.0 30.0 16000.0`): take
w1_gazebo, 16 kJ, commit on zone 2, 0.00 % stale scans (W1 is light to
render), shown in real time. The recording starts on the GUI's wide
default view at take-off, then the camera joins the drone.
