# Report analyses, 31 Oct to 1 Nov 2026

Offline analyses written for the final report (`docs/report/`), all
from the logs of the final campaigns (210 missions,
`2026-10-23_campaign_w{1,2,3}_final`) and of the provoked zone loss
(13 missions, `2026-10-24_s95_return`). No flight was made for them.
Each script states its question in its header and prints the numbers
the report quotes; the `.txt` next to it is that output.

Run from this directory with ROS and the workspace sourced (the map
panels import `uav_core`): `bash run.sh SCRIPT.py`. The first run of
`build_cache.py` writes a compact per-mission cache to `cache/`
(ignored by git, about 25 MB); every script then reads it in seconds.

| script | question | headline result (report section) |
|---|---|---|
| `a_alpha.py` | where is the coverage optimum in alpha, world by world? | interior plateau 0.5 to 0.75 after Holm correction, not 0.75 alone on every world (6.5) |
| `b1_energy_prediction.py`, `b1_counts.py` | is the energy C1 charges in flight above the energy measured? | above in 2530 of 2535 frontier transits, 174 of 180 final returns; returns and landing under-charged in 5 of 180, by at most 488 J (6.3) |
| `b2_second_zone.py` | how often did a decision rely on a single zone? | 70 of 1710 decisions on a discovered zone (4 %) (6.6) |
| `b3_b4_b6_decisions.py` | what do C1 and C2 do at each decision, at what cost? | C1 idle below 30 % of the budget spent, decisive beyond 70 %; median cycle 29, 103, 198 ms on W1, W2, W3 (6.4, 6.5, 6.7) |
| `b5_guard_final.sh`, `b5_where.py` | does the confirmation guard hold on the final W1 maps? | no: 10 of 70 maps keep a dangerous landable cell, at the raised north corner of the 3 deg ramp; no landing there (6.2) |
| `b7_mission_metric.py`, `b7_mechanism.py`, `b7_w2_zones.py` | how fast and how much landing area does each configuration find? | confirmed area rises with alpha (W3 328 to 625 m2, rho +0.44); no trend in the energy to the first zone (6.5) |
| `commit_cell.py` | was the cell under the landing commit still landable? | 149 of 150 commits; the other on a refused road cell, safe in the truth (6.6) |
| `landings.py` | where did every C1 mission land, against the truth? | 149 of 150 on ground landable by design, one on forest floor (6.2) |
| `fig_worlds.py` | `r4_worlds_grid.png`: the three worlds, render, truth, typical final map | Figure 5 |
| `fig_film.py` | `r5_mission_film.png`: one W3 mission, map frames, guarantee, registry | Figure 17 |
| `fig_c1_decision.py` | `r6_c1_decision.png`: C1 on one real decision | Figure 2 |
| `fig_analyses.py` | `r7_analyses.png`: charge against measurement, rejections, cycle time, landing area | Figure 11 |
| `fig_alpha_w2.py` | `r8_alpha_w2.png`: final maps of two filmed W2 flights at alpha 0.1 and 0.9 | Figure 14 (illustration; no campaign flew these values) |

Shared code: `common.py` (loader and cache), `style.py` (palette and
the perception replay of the map panels), `build_cache.py`.

Figure numbers are those of `report_draft_v3.docx`.
