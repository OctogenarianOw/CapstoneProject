# Observation thresholds and frontier definition, 19 Oct 2026 (S6, S11)

Offline measurements on the reference map data_survey_v6.npz against the
analytic truth of worlds/validation_truth.npz. No flight.

## 1. n_min and var_max (measure.py, measure.txt)

Elevation error of flat cells against the truth, per bin of observation
count, after removing the constant offset (+0.368 m) as validate_map.py
does. Flat ground isolates estimation noise from the P2 edge bias.

| observations | cells | RMS error |
|---|---|---|
| 1 to 4 | 10 | 2.3 to 2.9 m |
| 5 to 9 | 2 | 0.35 m |
| 10 and more | 2901 | 0.07 to 0.24 m per bin |

The low-count cells all sit at E +28.5, east of the refused ramp, in its
occlusion shadow: the few returns that land there come from the ramp.
Few observations mark a masked cell, not only a noisy one.

Chosen from this: n_min = 10, the first count with every bin under the
0.25 m target of S5, and var_max = 2e-3 m2, the median fused variance at
10 to 19 observations, which also refuses cells seen only from far.
Weak on sample size: a survey map holds 12 flat cells under 10
observations. To be measured again on exploration maps.

## 2. Frontier definition (frontier_ring.py, frontier_ring.txt)

| definition | frontier cells on v6 |
|---|---|
| S11: OBSERVED cell next to UNKNOWN | 53 |
| any seen cell next to UNKNOWN | 404 |

87 % of the boundary is separated from OBSERVED by a SPARSE ring, 1 cell
wide at the median, 3 at most: cells at long range gather observations
slowly. The S11 wording would miss most of the boundary and could end
exploration with unknown ground left. Chosen: a frontier cell is any
seen cell next to an UNKNOWN one, as S6 allows separate thresholds for
frontiers and analysis. Viewpoints stay OBSERVED and reachable.

## 3. Frontier catalogue on v6 (frontiers_on_v6.py, frontiers_on_v6.txt)

Survey map, validation world: the only unknown ground left is the 15
cells of the ramp shadow, giving 4 pieces, all with gain 15.

Early flight, v6 cut to the 18 m footprint seen from take-off: before
splitting, the whole ring was one cluster of 140 cells whose centroid was
the take-off point, so its single viewpoint was arbitrary. With clusters
cut into pieces of at most 10 m: 12 pieces all around, viewpoints on the
frontier at 17.4 to 17.7 m, gains 523 to 602. 14 ms on the 100 x 100
grid.

    cd ~/ros2_ws/droneAfter/experiments/2026-10-19_observation_thresholds
    python3 measure.py ../../data_survey_v6.npz
    python3 frontier_ring.py ../../data_survey_v6.npz
    python3 frontiers_on_v6.py ../../data_survey_v6.npz
