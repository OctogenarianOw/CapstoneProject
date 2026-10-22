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

To be filled after the flights.
