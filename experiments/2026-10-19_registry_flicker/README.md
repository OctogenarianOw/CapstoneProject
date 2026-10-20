# The touchdown point of a large zone, and the passes counted around it, 19 Oct 2026

Question: why does a confirmed landing zone fall back to CANDIDATE with
the reason "1 pass, 2 needed" while nothing was observed against it?

It mattered for C1. In the W1 campaign (`2026-10-19_campaign_w1`, code
7dc4741) 31 of the 60 C1 missions broke I1, and every first violation
came within 3 s of such a demotion of the zone C1 relied on. In the
190 m re-flight (`2026-10-19_big_area`) an abort first described as an
S9.5 sequence had the same trigger.

## Cause, from the logs

The registry counts the passes of I4 around the zone's touchdown
point, the zone cell farthest from refused ground. `cycles.jsonl` logs
that point and the pass count of every zone at every cycle
(`live_flicker.py`). In a large flat zone hundreds of cells lie beyond
the 12-cell cap of that distance, and the farthest one moves with every
change of the map edge:

| mission | touchdown moves | over 5 m | largest | pass-count demotions |
|---|---|---|---|---|
| live demo 200 kJ (d873aa8) | 46 | 16 | 63.7 m | 5 |
| first autonomous mission (186e516) | 37 | 9 | 67.9 m | 4 |
| campaign B3a075_r01 (7dc4741) | 26 | 6 | 32.4 m | 4 |
| 190 m re-flight (db7dc55) | 58 | 24 | 159.8 m | 3 |

Example, B3a075_r01 at 49.9 s: zone 2, 2278 m2, touchdown from (-4.5,
1.5) to (22.5, 19.5), next to the aircraft; passes around the new point
1; zone demoted; the return falls back to the take-off point, I1 at
-0.57 kJ.

## Fix (c7fc2aa) and measurement

A known zone keeps, among the cells within one cell of the best capped
distance, the one nearest its previous touchdown; a new zone still takes
the farthest cell; the inscribed radius is unchanged. One cell is the
resolution of the distance. Rejected: a pass count that never
decreases, which changes what I4 means instead of removing the cause.

W1, c1_c2, alpha 0.5, 200 kJ (the budget never binds, the mission
covers W1, the large zone around take-off is tracked for the whole
flight), fresh PX4 with reset parameters, 0 failsafes, `bash run.sh
LABEL 2`:

| flights | git | touchdown moves | over 5 m | largest | pass-count demotions | coverage |
|---|---|---|---|---|---|---|
| `before_r1`, `before_r2` | 9bb744a | 34, 37 | 5, 12 | 44.0, 48.2 m | 2, 3 | 100, 100 % |
| `after_r1`, `after_r2` | c7fc2aa | 3, 4 | 0, 0 | 4.5, 1.4 m | 1, 0 | 100, 100 % |

Offline, replaying the registry on the 10 s map snapshots of earlier
missions (`replay_flicker.py`, same input, installed module against the
worktree one): moves over 5 m 5 -> 2 on the live demo, 4 -> 0 on the
first autonomous mission. The replay does not reproduce the demotions
themselves, which need the 1 Hz map the snapshots do not keep.

## What remains

One demotion in `after_r1`, at 30.8 s, of another kind: while zone 2
was still growing (928 to 1231 m2, below the cap), a cell genuinely
clearer by more than one cell appeared and the touchdown moved 4.5 m,
from (-3.5, 0.5) to (0.5, 2.5). Around the new point the trajectory had
not left the 18 m radius between the two entries, so the hysteresis
counted 1 pass; the zone was confirmed again at 36.8 s. Removing that
case means deciding what a pass is for a zone that changes shape, a
definition of I4, left to Dorian (counting passes around a point that
never decreases, or around the zone's cells, are the two candidates).

## The W1 campaign with this fix, 20 Oct

`2026-10-20_campaign_w1_fix`, the 80 missions of the first campaign with
c7fc2aa as the only change: C1 missions breaking I1 31/60 -> 0/60,
pass-count demotions 150 -> 5. The 5 left are the case above; one of
them sent B1_r10 back to the take-off point and broke I1 7 times.

## I4 option b (d115a97), 20 Oct

Decided by Dorian on 19 Oct: the pass count of a known zone never
decreases; a pass flown over a zone stays counted when its touchdown
moves. A unit test reproduces the `after_r1` case (1 pass before the
change, 2 after, and a cell short of OBSERVED still demotes the zone).

Three flights, same protocol, code d115a97, `bash run.sh i4passes 3`,
fresh world, 0 failsafes:

| flights | git | touchdown moves | over 5 m | largest | pass-count demotions | coverage | I1 violations |
|---|---|---|---|---|---|---|---|
| `after_r1`, `after_r2` | c7fc2aa | 3, 4 | 0, 0 | 4.5, 1.4 m | 1, 0 | 100, 100 % | 0, 0 |
| `i4passes_r1..3` | d115a97 | 2, 3, 4 | 0, 0, 0 | 1.0, 2.8, 2.0 m | 0, 0, 0 | 100, 100, 100 % | 0, 0, 0 |

Did the change act in these flights? `counterfactual_passes.py`
recomputes, from the 1 Hz log, the count the registry would have had
before d115a97, around each logged touchdown over the track so far. On
the four flights before the change it equals the logged count at all
588 zone-cycles, which checks the script. On the three new flights it
also equals it at all 578: no touchdown moved away from the passes
already counted, so the change never acted and prevented no demotion.

So these flights show no regression (full coverage, no violation, the
same touchdown behaviour), not the effect. The effect rests on the unit
test; its frequency in flight is 5 demotions in 80 missions of the
campaign above, about one mission in sixteen, too rare to observe in
three. The evaluation campaign (H) will count it.

## Files

    run.sh                    N missions of one label, W1, 200 kJ
    before_r*/ after_r*/ i4passes_r*/
                              setup.txt, px4.log, node logs, analysis.txt,
                              missions/<date>/
    live_flicker.py           touchdown moves and demotions as flown
    replay_flicker.py         the registry replayed on map snapshots
    counterfactual_passes.py  pass counts without I4 option b, from the log
    live_flicker.txt          live_flicker output for the four flights of 19 Oct
    live_flicker_20oct.txt    the same with the three I4 flights
    counterfactual_passes.txt counterfactual_passes output, seven flights

    cd ~/ros2_ws/droneAfter/experiments/2026-10-19_registry_flicker
    python3 live_flicker.py before_r*/missions/* after_r*/missions/* i4passes_r*/missions/*
    python3 counterfactual_passes.py before_r*/missions/* after_r*/missions/* i4passes_r*/missions/*
