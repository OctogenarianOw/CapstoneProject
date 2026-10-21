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

12 sessions, all at the first attempt, 09:50 to 10:29 (`flights.log`,
`results.txt`). The 60 s check passed. One step of the clock, +0.461 s
(forward, unlike the backward steps of 21 Oct), 18.5 s into
A_before_r1, during the take-off; none in the 11 other flights. By the
rule above A_before_r1 does not count; it was not flown again, because
the verdict rests on the after flights, all clean, and its values equal
r2 and r3 to 0.001.

| | before (3 runs) | after (3 runs) | criterion |
|---|---|---|---|
| scans at the scan time | 0 % | **100 %**, no fallback | > 95 %: met |
| plates, A | +0.437, +0.437, +0.438 | **-0.248, -0.264, -0.237** | \|A\| < 0.1: **failed** |
| W1 best ramp shift | (-1, 0) x 3 | **(0, 0) x 3** | (0, 0): met |
| W1 ramp RMS at (0, 0) | 0.340 to 0.353 m | **0.173 to 0.183 m** | lower: met |
| W1 flat RMS | 0.244 to 0.256 m | **0.300 to 0.311 m** | < 0.25 m: **failed** |
| W1 recall | 93.7 to 93.9 % | **60.0 to 64.0 %** | >= 91.7 %: **failed** |
| W1 severe false accepts | 2, 2, 4 | 0, 0, 0 | no more: met |

**The fix fails on a stable clock, so it is not merged** (merge rule
above). The clock was not the cause of the failure of 21 Oct: with no
step and no fallback, every after flight fails the same three tests,
more consistently than on 21 Oct (A from -0.24 to -0.26 against -0.01
to -0.67, flat RMS 0.30 to 0.31 against 0.37 to 0.55 m).

What stands, measured twice now: the pose taken at the scan time
removes P5 on the ramps of W1, and with it the severe false accepts of
fresh surveys (2, 2, 4 with the code of dronePropre, 0, 0, 0 with the
fix). The S8 barrier fails on fresh surveys without it.

What fails: the spill on the plates changes side and keeps about 57 %
of its size, so the correction overshoots, by about 0.065 s if the
spill scales with the lag (0.115 s gave +0.44); and the map gets worse
on flat ground, where an along-track shift changes nothing. The second
points to the attitude rather than the position. **Hypothesis, not
measured**: the 0.085 s calibration term was measured on the position
(the PX4 estimate ahead of the Gazebo truth), but the fix applies it to
the attitude too. An attitude 0.085 s old during the accelerations of
a survey tilts the scan by degrees, tens of centimetres at 15 m, which
would raise the flat RMS and cut the recall. Next, offline, one change
at a time, on the bags of 2026-10-21_p5_latency: the term on the
position only, then the term set to zero.
