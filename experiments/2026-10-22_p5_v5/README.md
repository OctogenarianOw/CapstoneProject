# P5 fix v5 in flight, 22 Oct 2026

v4 (position at the scan stamp minus 0.050 s, attitude last received,
`2026-10-22_p5_v4`) passed every W1 test in flight (r1: ramps 0.34 ->
0.17 m, shift (0, 0), flat 0.243 -> 0.233 m, recall 94.4 -> 95.5 %,
severe 2 -> 0) but left the plate spill at A = +0.135 and +0.127, over
the 0.1 of the criterion: the term is short. Between +0.13 at 0.050 s
and -0.25 at 0.085 s (v3, 2026-10-22_p5_stable_clock), A crosses zero at
0.060 to 0.062 s. v5 (`p5-v5`, 1dc42c2): the term set to 0.060 s,
nothing else.

## Set-up

The protocol of 2026-10-22_p5_v4, after sessions only: `run.sh 3` flies
3 A (plates) and 3 B (W1 survey) with pose_at_scan_time true, from the
worktree `~/uav_p5v5`. The before sessions are those of 2026-10-22_p5_v4
(pose_at_scan_time false), flown the same evening on the same clock with
the same protocol: with the parameter false, v4 and v5 run the same code
path (the latest pose), so flying them again would measure the same
thing twice. `analyse.py` reads both.

## Criterion and merge rule, fixed before the flights

Those of 2026-10-22_p5_v4 (and of 2026-10-22_p5_stable_clock), against
its before sessions. If the criterion holds, `p5-v5` is merged into
dronePropre with /clock bridged by sim_session.sh, and the W3 campaign
is flown on it; if not, nothing is merged and the campaign is flown
without it.

## Results

To be filled after the flights.
