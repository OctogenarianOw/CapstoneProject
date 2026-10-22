# Evaluation campaign on W3, 22 Oct 2026

Gates E and F on the third world of spec 5.1, W3, the large mixed world
Dorian asked for (`tools/make_world_w3.py`: road, town, forest, scrub,
rock, hills; 240 m, 7 landable components by design). Same seven
configurations, same metrics and statistics as the W1 and W2 campaigns
(`run.sh`, `summarise.py`), so the three tables read the same.

## What differs from the W1 and W2 campaigns

The code under test is dronePropre after the changes of 22 Oct, each
measured on its own before being merged:

- I4 per cell (`2026-10-22_i4_cells_flights`): passes counted per
  cell, a zone confirmed on a whole footprint passed twice, C1 targeting
  only such cells. W1 and W2 were flown with passes around one touchdown
  point per zone;
- the replanning on an unreachable target (`2026-10-22_reselect_flights`):
  without it every W3 mission ended on the first viewpoint a roof
  blocked.

The W1 and W2 results stay as recorded, measured before these changes.
The commit is in every mission's config.json.

## Set-up

`run.sh N`: W3, map 240 m, area 160 m, 80 kJ (about two thirds of a
full coverage), reserve 10 %, headless, fresh PX4 world and reset
parameters for every mission, the WSL clock checked before and watched
in every mission. B0 (nearest frontier, no energy constraint), B1
(information gain, return at 30 % of the capacity), B3 (C1 and C2) at
alpha 0, 0.25, 0.5, 0.75 and 1. Interleaved, run index outer: N = 5 on
22 Oct, raised to 10 later by running `run.sh 10`, which skips the
missions done.

## Criteria, fixed before the flights

- Gate E: no I1 violation in any of the C1 missions (B3, every alpha).
  Reported against B0 and B1: missions with a violation, smallest I1
  margin, coverage, energy left (Mann-Whitney U, rank-biserial r).
- Gate F: a monotone and interpretable effect of alpha on at least one
  metric over the B3 missions (Spearman), and the medians per alpha.
- Safety (S8, I5): no landing on a cell severe in the truth; every
  landing of a C1 mission on a registry zone.

With N = 5 the tests are weak; the tables are reported as they are,
labelled with N, and completed at N = 10.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_campaign_w3
    bash run.sh 5
    python3 summarise.py > summary.txt      # ROS and the workspace sourced

## Results

To be filled after the flights.
