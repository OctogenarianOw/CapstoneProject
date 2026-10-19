# Landing zone registry replayed on the survey, 19 Oct 2026 (S9)

replay.py feeds the mission log of
2026-10-19_live_map/survey_observe through uav_core/slz_registry.py: the
vehicle position of every 1 Hz cycle, and the latest map snapshot (every
10 s) through the same detection as the manager (footprint 1.5 m).

## Passes counted from detection: no zone confirmed

First version of the registry, passes counted from the first detection
of a zone onwards (not committed; numbers from its run): 16 zones
detected over the flight, 0 confirmed. The 391 m2 zone first detected at
67 s had 0 passes although the aircraft had flown over it: a zone only
appears once its cells are well observed, so passes flown before that
moment, which built the evidence, were lost.

## Passes counted over the whole track (committed): 5 confirmed

Same log, passes recomputed at every cycle over the whole vehicle track
(replay_survey.txt): 5 zones confirmed, all central, 2 to 31 m2, 4
passes each. Still not confirmed:

| zone | area | passes | blocking criterion |
|---|---|---|---|
| 6 | 672 m2 | 1 | cells not all OBSERVED |
| 8 | 391 m2 | 1 | 1 pass, 2 needed |
| 4 | 176 m2 | 1 | 1 pass, 2 needed |

Zones 8 and 4 lie at the edge of the survey: going from the end of one
line to the start of the next, the aircraft never leaves the sensor
radius, which by definition is one pass. Zone 6, the large flat area,
holds at least one cell short of OBSERVED, at its border: the criterion
"every cell OBSERVED" is applied to zones built from cells that were
never required to be OBSERVED. The next change gates the classification
itself on OBSERVED (the variance gate S8 asks for), measured separately.

After landing (from 78 s) small zones appear and retire: the map
degradation from the ground noted in 2026-10-19_live_map.

## Detection restricted to OBSERVED cells (2026-10-19_observed_gate)

The default replay now uses world_model.detect_landing, the manager's
detection, gated on OBSERVED cells (replay_survey.txt; the ungated run
is kept as replay_survey_ungated.txt). Zone 6 shrinks from 672 to
632 m2, the 40 cells short of OBSERVED leaving it, and its blocking
criterion becomes "1 pass, 2 needed": the cell criterion now holds by
construction. Confirmed zones: 5 in both runs, as the edge zones keep a
single pass on this survey geometry.

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-19_registry_replay/replay.py \
      experiments/2026-10-19_live_map/survey_observe/missions/*
    python3 experiments/2026-10-19_registry_replay/replay.py \
      experiments/2026-10-19_live_map/survey_observe/missions/* --ungated
