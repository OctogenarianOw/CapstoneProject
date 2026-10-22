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

6 after sessions at the first attempt, 20:07 to 20:27, no clock step,
tick at 10000 us (`flights.log`, `results.txt`, before sessions read
from 2026-10-22_p5_v4).

| | before (3, v4 experiment) | after, v5 (3) | criterion |
|---|---|---|---|
| plates, A | +0.443 to +0.445 | **+0.012, +0.051, +0.067** | under 0.1: met |
| scans at the scan time | 0 % | 100 % | over 95 %: met |
| W1 best shift | (-1, 0) x 3 | (0, 0) x 3 | (0, 0): met |
| W1 ramp RMS at (0, 0) | 0.339 to 0.340 m | 0.162 to 0.165 m | lower: met |
| W1 recall | 93.4 to 94.7 % | 94.8 to 95.3 % | 91.4 % or more: met |
| W1 severe false accepts | 2, 3, 3 | 0, 0, 0 | no more: met |
| W1 flat RMS | 0.243, 0.201, 0.231 m | 0.246, 0.236, **0.254** m | under 0.25 m in every flight: **failed by 4 mm** |

**v5 fixes P5 on the plates and on W1, and fails the criterion as
written, by one flight at 0.254 m of flat RMS. Not merged** (rule
above), and the W3 campaign is flown without it.

For Dorian, not decided here: the 0.25 m threshold of S5 lies inside
the flight-to-flight spread of this metric. The code without the fix
broke it too on 22 Oct (0.256 m in B_before_r1 of
2026-10-22_p5_stable_clock, 0.250 m in its r3); its six flights of the
day range 0.201 to 0.256 m, v5's three 0.236 to 0.254 m. A
non-inferiority test against the before sessions would be the fitting
criterion; changing it after the results is not for this experiment to
do. Either a decision on these data, or a new series of W1 surveys with
that criterion fixed before.
