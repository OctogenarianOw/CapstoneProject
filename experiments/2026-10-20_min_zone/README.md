# Confirm only zones that hold a whole footprint, 20 Oct 2026 (S8, S9)

`2026-10-20_footprint_p4` found that the barrier of S8, zero severe
false accepts, does not hold on mission maps: at the 1.5 m footprint in
use, 14 of the 80 last maps of `2026-10-19_campaign_w1` hold one, in the
rough zone of W1, where the map smooths 0.5 m steps to under 0.2 m.
Every one lies inside a detected zone of 2 to 6 cells.

Why it matters beyond the map: the return field targets the nearest
landable cell of any CONFIRMED zone, so a confirmed island is a landing
target. `confirmed_small.py` reads the registry states logged at 1 Hz:
**the registry confirmed a zone of less than 9 m2 in 28 of the 80
missions of the first campaign and 15 of the 80 of the second**, all in
the rough zone (N +16 to +22, E -3 to +2). None was landed on.

## Candidate rules, measured offline

`measure.py`: the manager's detection (OBSERVED cells, footprint 1.5 m)
on v6 and on the last map of each mission, a rule applied to the
detected zones, severe false accepts left and recall against the
unfiltered truth. The rule was chosen on the first campaign and checked
on the second, flown independently.

| rule | v6 severe | campaign 1: maps with severe, cells | campaign 2: maps with severe, cells | recall median, 1 / 2 |
|---|---|---|---|---|
| none (in use) | 0 | 14, 14 | 17, 22 | 84.8 / 84.7 % |
| zone of at least 4 cells | 0 | 5, 5 | 12, 17 | unchanged |
| zone of at least 7 cells | 0 | 0, 0 | **2, 2** | unchanged |
| zone of at least 9 cells | 0 | 0, 0 | 0, 0 | unchanged |
| **zone holding one whole footprint element** | 0 | **0, 0** | **0, 0** | unchanged |

(`measure_campaign_w1.txt`, `measure_campaign_w1_fix.txt`.) The
truth of W1 at this footprint is one connected landable zone of 2308
cells, so no rule here refuses a real opportunity on W1; the cost shows
on a world with small true zones, below.

## Decision

A zone is confirmed only if it holds one whole footprint element of
landable cells: the footprint test of S8 applied to the landable cells
themselves (`slz_registry`, criterion "no whole footprint of landable
cells"). Detection is unchanged; the islands stay CANDIDATE, visible
and logged, never relied on by C1.

Why this rule and not a count of cells: the count that cleared the
first campaign, 7, let 2 severe cells through on the second, so a count
is a fit to one sample; and a count says nothing of shape, a strip one
cell wide can hold any number. The footprint rule has a physical
reading: the aircraft must fit in the zone with a footprint of margin
of landable ground around the touchdown, not only on cells that just
pass. It held on both campaigns.

Price: the smallest isolated flat platform that can be confirmed grows
from 5 m to 7 m wide at 1 m resolution (S8: a 5 m platform yields one
landable cell, a 7 m platform the 3 x 3 cells that hold the element).
That is a lost opportunity, the tolerable error of S8, against a
landing target inside ground the map is known to smooth.

What this does not do: it does not fix the map. The cause of the
smoothing in the rough zone is not measured; P5 (a one-cell offset on
the ramps) points to horizontal misplacement of the points, to be
measured as its own change.

Unit test (`test_island_without_a_whole_footprint_is_never_confirmed`):
a 2 x 3 island with two passes and OBSERVED cells is CONFIRMED before
the change and stays CANDIDATE after; a 3 x 3 block next to it is
confirmed. In flight, the evaluation campaign will count the small
confirmed zones, 15 and 28 in 80 before.

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-20_min_zone/measure.py data_survey_v6.npz \
        --runs experiments/2026-10-19_campaign_w1/runs
    python3 experiments/2026-10-20_min_zone/confirmed_small.py \
        experiments/2026-10-20_campaign_w1_fix/runs/*

Files:

    measure.py                         rules against severe false accepts
    measure_campaign_w1.txt            its output, first campaign
    measure_campaign_w1_fix.txt        its output, second campaign
    confirmed_small.py                 small zones confirmed in flight
    confirmed_small_campaign_w1_fix.txt  its output, second campaign
