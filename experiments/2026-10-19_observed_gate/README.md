# Landing detection restricted to OBSERVED cells, 19 Oct 2026 (S8, I4)

S8: the variance gate of classify_cells "exists in the signature but is
not used yet; it must be, it is the link with I4". measure.py runs the
evaluation of tools/validate_slz.py (footprint 1.5 m, truth detected
with the same footprint, severe = truth past twice a threshold) with the
map detection as committed and with the gate fed OBSERVED cells only.

| map | gate | recall | accepted | benign | severe |
|---|---|---|---|---|---|
| v6, reference | none | 94.2 % | 2219 | 46 | 0 |
| v6, reference | OBSERVED | 94.2 % | 2219 | 46 | 0 |
| v4 | both | 86.7 % | 2067 | 56 | 9 |
| v3 | none / gate | 50.5 / 49.7 % | 1369 / 1347 | 112 / 111 | 91 / 88 |
| v1, v2 | both | unchanged | | | |
| v5 | none / gate | 48.9 / 0.0 % | 1165 / 0 | 35 / 0 | 1 / 0 |

On the reference map the gate changes nothing: every accepted cell of a
survey map is already OBSERVED. v5 comes from an earlier version of the
mapping whose variances sit above 2e-3 m2; it is not a reference.

What the gate changes is the registry: on the replayed flight of 19 Oct
the largest zone loses 40 border cells short of OBSERVED, and the
criterion "every cell OBSERVED" holds by construction instead of
blocking it (2026-10-19_registry_replay).

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-19_observed_gate/measure.py data_survey_v*.npz
