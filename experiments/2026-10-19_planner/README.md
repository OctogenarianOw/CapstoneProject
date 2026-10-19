# Planner on map v6, 19 Oct 2026 (S12)

Offline check of uav_core/planner.py on data_survey_v6.npz, at the
8 m cruise altitude, 2 m vertical margin, 1.25 m inflation.

| quantity | value |
|---|---|
| traversable cells | 6181, 88.7 % of the seen cells; the rest is the inflated band along unknown ground |
| highest seen terrain | 3.50 m above the EKF origin (the 4 m cube, shrunk by P2) |
| outbound field, 1 source | 11.9 ms |
| return field, 4 sources | 8.9 ms |
| plan to N +25 E +25 | 1 leg, 35.36 m, 0.6 ms |

At 8 m the validation world blocks nothing, so every plan is one leg;
the wall cases are covered by src/uav_core/test/test_planner.py.

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-19_planner/measure.py data_survey_v6.npz
