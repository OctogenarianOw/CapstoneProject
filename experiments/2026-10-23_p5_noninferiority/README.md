# P5 fix v5 on W1 surveys, non-inferiority on the flat, 23 Oct 2026

v5 (`p5-v5`, 1dc42c2: position at the scan stamp minus 0.060 s,
attitude last received) removed P5 on the plates and on W1 on 22 Oct
(`2026-10-22_p5_v5`: A +0.44 to +0.01..+0.07, shift (0, 0), ramps
0.34 to 0.16 m, recall kept, severe false accepts 2-3 to 0), and failed
its criterion by one flight whose flat RMS was 0.254 m against 0.25 m
in every flight. The code without the fix broke that threshold too that
day (0.256 m); its six flights ranged 0.201 to 0.256 m. A threshold
inside the flight-to-flight spread tests the noise, not the fix.

Decision of Dorian, 23 Oct: fly W1 surveys again with a
non-inferiority criterion on the flat fixed before the flights, rather
than read the 22 Oct data with a criterion chosen after them.

## Set-up

`run.sh 3`: the protocol of `2026-10-22_p5_v5`, W1 survey (B) only,
3 before and 3 after interleaved (before, after, before, ...), fresh
PX4 and reset parameters every time, /clock bridged in both, the fix
read from the worktree `~/uav_p5v5` (1dc42c2) through PYTHONPATH,
before and after differing by the one parameter pose_at_scan_time. The
WSL clock checked 60 s before and watched in every flight. About 25
min. The plates (A) are not flown again: v5 passed them in 3 of 3.
Since the branch point of `p5-v5`, dronePropre has not changed
cloud_world.py, so the flights test the code a merge would bring.

## Criterion, fixed before the flights

Only flights with no clock step count. In every after flight: more
than 95 % of the scans at the scan time; best ramp shift (0, 0); ramp
RMS at (0, 0) under the lowest before; recall no lower than the lowest
before minus 2 points; no more severe false accepts than the fewest
before. These are the items of 22 Oct, unchanged.

The flat, now a non-inferiority test: the median flat RMS after at
most the median flat RMS before plus delta = 0.02 m. Why 0.02 m: the
flat RMS of the code without the fix spread over 0.055 m across six
flights of one day (0.201 to 0.256 m); a difference under about a third
of that spread is not one the protocol can tell from noise, and 2 cm
is a twelfth of the 0.24 m step a cell must exceed to be refused, so it
changes no classification that matters. The absolute 0.25 m of S5 is
reported, not a condition, since the code without the fix fails it too.

## Merge rule, fixed before the flights

If every item holds, `p5-v5` is merged into dronePropre with /clock
bridged by sim_session.sh (part of the branch), the build is checked,
and the W3 campaign is flown on the result. If an item fails, nothing
is merged and P5 stays a declared limitation.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_p5_noninferiority
    bash run.sh 3
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

To be filled after the flights.
