# P5 fix v3 on a stable clock, 22 Oct 2026

`2026-10-21_p5_fix` measured the fix of P5 (pose taken at the scan
time, branch `p5-interp`, v3 at 548aa15) while the WSL system clock
stepped back by about 0.7 s every 32 s (spec trap 22). On the ramps of
W1 the fix removed P5 and the severe false accepts, but the flat RMS
rose from 0.22-0.25 to 0.37-0.55 m and the recall fell from 93-94 to
51-56 %, and on the plates the spill turned to the other side. The
clock steps could explain it, nothing measured that. WSL was restarted
on 22 Oct (boot 09:39) and a 70 s check showed no step. This repeats
the measurement on that clock.

## Set-up

The protocol, worlds, flights and tools of `2026-10-21_p5_fix`,
unchanged: `run.sh 3` flies 3 runs of each of the four sessions
(A plates on world `pits`, B the W1 survey, each before and after),
interleaved, fresh PX4 and reset parameters every time. The one change
between before and after is the parameter `pose_at_scan_time` of the
fix's cloud_world. Two differences with 21 Oct, neither in the code
under test:

- the fix is read from the worktree `~/uav_p5`, detached at 548aa15,
  so development in `~/uav_dev` cannot reach the flights;
- `clock_watch.py` checks the clock for 60 s before the flights
  (no flight on a step) and logs any step during each flight
  (`clock.log` in each run directory).

The before runs are flown again rather than reused, so that before and
after share the day and the clock.

## Criterion, fixed before the flights

Only flights with no clock step count; a flight with a step is
reported and flown again.

The criterion of 21 Oct: in every flight after, |A| under 0.1 with more
than 95 % of the scans at the scan time; on W1 a best ramp shift of
(0, 0) with a ramp RMS at (0, 0) lower than before; the flat RMS and
the severe false accepts no worse. Made explicit today, because the
recall fell on 21 Oct without the criterion naming it: in every W1
flight after, the flat RMS under the S5 criterion of 0.25 m, the
recall no lower than the lowest before minus 2 points, and no more
severe false accepts than the fewest before.

## Merge rule, fixed before the flights

Dorian delegated the decision on 22 Oct. If the criterion passes,
`p5-interp` is merged into `dronePropre`: the S8 barrier (zero severe
false accepts) failed on the three fresh surveys of 21 Oct with the
current code and held with the fix, so a map without it cannot carry
the safety claim of the thesis. The W1 and W2 results already recorded
stay as they are, labelled as measured before the fix, and campaign H
is flown after it. If the criterion fails on a stable clock, the fix is
not merged and the failure is diagnosed before anything else changes.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_p5_stable_clock
    bash run.sh 3
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

To be filled after the flights.
