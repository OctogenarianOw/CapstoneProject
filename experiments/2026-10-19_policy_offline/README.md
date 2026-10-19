# C1 and C2 offline on the take-off situation, 19 Oct 2026 (S13, S14)

measure.py runs uav_core/mission_policy.py on map v6 cut to the 18 m
footprint seen from take-off, the take-off point as the only zone
(provisional, S15.3), energy model of 19 Oct, dwell 2 s, no reserve.

12 candidates, one evaluation in 2.2 ms. Every outbound and return path
is a single leg on this open ground. Per candidate: E_go 1.93 to
2.03 kJ (path plus the 2 s dwell, 571 J), E_return 1.36 to 1.46 kJ,
E_total 6.49 to 6.68 kJ, of which E_land 3.20 kJ.

| budget B | admissible | chosen, any alpha |
|---|---|---|
| 20 kJ | 12 | N +10.5 E +10.5, G 1.00, M 0.68 |
| 7 kJ | 12 | same, M 0.07 |
| 6.5 kJ | 1 | same, M 0.00 |

C1 bites as the budget shrinks. Here the same candidate wins for every
alpha: it has both the largest gain (514, a diagonal piece reaching the
world corner) and the smallest E_total. The trade-off alpha governs
appears only when gain and margin disagree, which the missions measure.

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-19_policy_offline/measure.py data_survey_v6.npz
