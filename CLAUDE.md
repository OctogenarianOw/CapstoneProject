# CLAUDE.md

## Read this first

Read `docs/prompt-maitre-projet-uav.md` in full before doing anything
else. It is the complete specification of this project: research
question, contributions, component-by-component requirements with
acceptance criteria, evaluation protocol, delivery plan, trap
catalogue, and operational appendix (layout, PX4 dependencies, launch
sequence, coordinate frames).

Do not look for `claude/etat-du-projet-uav.md`. It does not exist on
this machine: it lives in the claude.ai project knowledge base, which
you cannot see. Everything in it that is still true was folded into
Annexe A of the specification. Everything else was superseded.

## Project in one line

Master's thesis, UTS, defended December 2026. A simulated UAV explores
unknown terrain, detects geometrically safe landing zones, and keeps a
continuous guarantee that it can still reach one of them with the
energy it has left. ROS 2 Jazzy, PX4 SITL, Gazebo Harmonic, WSL2.

## Working rules

- Build before testing. `python3 -m pytest` imports from
  `install/`, not from `src/`, so a stale install makes passing code
  look broken. Always `colcon build --packages-select uav_core` then
  `source install/setup.bash` first.
- Never run `colcon build` from `~/ros2_ws`. Only from inside
  `droneAfter` or `droneBefore`, which are two separate workspaces.
- One change at a time, with a measurement before and a measurement
  after. Two simultaneous changes make the effect unattributable, and
  this project needs attributable effects for its results chapter.
- Measure before fixing. Several plausible hypotheses in this project
  have already been refuted by measurement. State clearly what is
  measured, what is assumed, and what is chosen.
- Build directly, do not wait for line-by-line approval, but write the
  reasoning down. Every module carries a header comment in English
  giving its role, the design choice made, the alternative rejected,
  and why. That comment is thesis defence material.
- Commit messages carry the measurement, not just the intent:
  `Fix footprint element to cover disc, severe false accepts 25 -> 0 on v6`,
  not `Fix footprint`.
- Code and comments in English. Conversation with the user in French.
- No double hyphens and no em dashes in prose written for the user.
- Write files with LF line endings, never CRLF.

## Before writing any PX4 or ROS code

Check the trap catalogue in section 8 of the specification. Every entry
is a bug that produced no error message and cost a session. The most
frequent: NaN versus zero in `TrajectorySetpoint`, PX4 timestamps in
microseconds since boot, the 1 Hz timesync topic, versioned PX4 topic
names, QoS mismatch producing silence, and Gazebo encoding missing
LiDAR returns as zero rather than infinity.

## Data

`data_survey_v6.npz` is the current reference map. Every published
figure is measured against it. Do not regenerate or overwrite it.
`/tmp/elevation_map.npz` is overwritten by any run of `cloud_world`,
so copy it to a versioned name immediately after each flight.
