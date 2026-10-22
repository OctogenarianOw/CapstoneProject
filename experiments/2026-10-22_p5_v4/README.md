# P5 fix v4 in flight, 22 Oct 2026

v3 (pose at the scan stamp minus 0.085 s, position and attitude)
failed on a stable clock (`2026-10-22_p5_stable_clock`). Offline
(`2026-10-22_p5_diagnosis`): on the plates the spill follows the term
on the position only and crosses zero near 0.045 to 0.050 s; on two W1
survey bags the interpolated attitude is what cut the recall (v3 61 %,
as in flight), and the position term with the attitude last received
kept it at 95 % while removing P5 and the severe false accepts. v4, on
branch `p5-v4` (32e4dcb, dronePropre with p5-interp merged and one
commit): position at the scan stamp minus 0.050 s, attitude last
received.

## Set-up

The protocol of `2026-10-22_p5_stable_clock`, unchanged: `run.sh 3`, 3
runs of each of four sessions interleaved (A plates on world pits, B
the W1 survey, each before and after), fresh PX4 and reset parameters
every time, /clock bridged in both, the fix read from the worktree
`~/uav_p5v4` through PYTHONPATH, before and after differing by the one
parameter pose_at_scan_time (false gives the latest pose, the behaviour
of dronePropre). The WSL clock is checked 60 s before (the kernel tick
too, `tools/wsl_clock_tick.py`, read in the evening chain) and watched
in every flight.

## Criterion, fixed before the flights (that of 2026-10-22_p5_stable_clock)

Only flights with no clock step count. In every flight after: |A| under
0.1 with more than 95 % of the scans at the scan time; on W1 a best ramp
shift of (0, 0) with a ramp RMS at (0, 0) lower than before; the flat
RMS under 0.25 m; the recall no lower than the lowest before minus 2
points; no more severe false accepts than the fewest before.

## Merge rule, fixed before the flights

If the criterion holds, `p5-v4` is merged into dronePropre, with
/clock bridged by sim_session.sh (without /clock the fix falls back to
the latest pose, counted but silent to a campaign), and the W3 campaign
is flown on it. If it fails, nothing is merged, P5 stays a declared
limitation, and the campaign is flown without it.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_p5_v4
    bash run.sh 3
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

12 sessions at the first attempt, 19:28 to 20:07, no clock step, tick
at 10000 us (`flights.log`, `results.txt`).

| | before (3) | after, v4 (3) | criterion |
|---|---|---|---|
| plates, A | +0.443 to +0.445 | **+0.126 to +0.135** | under 0.1: **failed** |
| scans at the scan time | 0 % | 100 % | over 95 %: met |
| W1 best shift | (-1, 0) x 3 | (0, 0) x 3 | (0, 0): met |
| W1 ramp RMS at (0, 0) | 0.339 to 0.340 m | 0.170 to 0.181 m | lower: met |
| W1 flat RMS | 0.201 to 0.243 m | 0.227 to 0.233 m | under 0.25 m: met |
| W1 recall | 93.4 to 94.7 % | 94.8 to 96.0 % | 91.4 % or more: met |
| W1 severe false accepts | 2, 3, 3 | 0, 0, 0 | no more: met |

**Not merged** (rule above): only the plate spill fails, and steadily,
a term short by about 0.01 s rather than noise. On W1 every test passes
in flight, as offline: P5 goes, the recall rises a point, the severe
false accepts of fresh surveys go. Between A = +0.13 at 0.050 s and
-0.25 at 0.085 s (v3), A crosses zero at 0.060 to 0.062 s: v5 flies
0.060 s (`2026-10-22_p5_v5`), against the before sessions of this
experiment.
