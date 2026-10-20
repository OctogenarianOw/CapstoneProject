# Footprint 1.2 m against 1.5 m, offline, 20 Oct 2026 (P4, S8, S16)

`2026-10-19_landing_p4` measured the landing error (p95 0.180 m) and
the airframe radius (0.386 m): the footprint of S16 is
2 (0.386 + 0.180) = 1.13 m, rounded up to 1.2 m, against 1.5 m in use.
Dorian approved the change on 19 Oct, to be measured offline first:
recall, severe false accepts, minimum zone. This is that measurement.
Nothing in `src/` was changed to make it.

`measure.py` runs the evaluation of `tools/validate_slz.py` and
`2026-10-19_observed_gate` with the footprint as the only difference:
truth detected with the same footprint, severe = truth past twice a
threshold, map detection on OBSERVED cells only as the manager does.
Maps: the survey maps v1 to v6 (v6 is the reference), and the last map
of each of the 80 missions of `2026-10-19_campaign_w1` (code 7dc4741),
80 independent flights where S8 asks for at least three before a zero
of severe false accepts counts. `inspect_severe.py` locates the severe
cells.

## Structuring element

`footprint_offsets` keeps a neighbour when the footprint disc reaches
inside it. At 1 m resolution the orthogonal neighbours start 0.5 m from
the centre, the diagonal ones 0.707 m. A 1.5 m footprint (radius
0.75 m) gives the 3x3 square, 1.2 m (radius 0.6 m) a plus of 5 cells.
Every footprint in (1.0, 1.414] m gives the plus, every one in
(1.414, 3.0] m the square.

| footprint | cells | rf | W min, S8 formula | W min, synthetic platform | cells refused around one bump |
|---|---|---|---|---|---|
| 1.5 m | 9 | 1 | 5.0 m | 5.0 m | 25 |
| 1.2 m | 5 | 1 | 5.0 m | 5.0 m | 21 |

The minimum zone does not change: rf stays one cell, and in a convex
zone a cell whose four orthogonal neighbours pass has its diagonal ones
inside too. The two elements differ only where a refused cell sits
diagonally, around isolated obstacles (4 cells of 25 recovered) and
along irregular edges. The registry criterion "inscribed radius at
least half the footprint" does not change either: at 1 m it takes the
values 0.5, 0.91, 1.5 m and up, and every candidate cell passes it with
either element.

## Detection against the truth

Survey maps (`measure.txt`):

| map | footprint | truth landable | recall | accepted | benign | severe |
|---|---|---|---|---|---|---|
| v6, reference | 1.5 m | 2308 | 94.2 % | 2219 | 46 | **0** |
| v6, reference | 1.2 m | 2324 | 95.2 % | 2266 | 51 | **2** |
| v4 | 1.5 m | 2308 | 86.7 % | 2067 | 56 | 9 |
| v4 | 1.2 m | 2324 | 88.4 % | 2133 | 67 | 12 |

v1 to v3 move the same way (severe 116, 89, 88 to 118, 94, 97); v5 has
no OBSERVED cell (`2026-10-19_observed_gate`).

Last map of the 80 missions of the first W1 campaign:

| footprint | maps with a severe cell | severe cells | recall, median | accepted, median | benign, median |
|---|---|---|---|---|---|
| 1.5 m | 14 | 14 | 84.8 % | 1964 | 7 |
| 1.2 m | 32 | 44 | 86.8 % | 2034 | 12 |

## Where the severe cells are

`inspect_severe.txt`.

v6 at 1.2 m: two cells at N -7.5, E -18.5 and -17.5, at the edge of
`ramp_ok`: truth slope 14.1 and 15.5 deg, truth step 0.50 and 0.55 m,
map slope 3.4 and 3.8 deg. That is the ramp edge where P5, the
one-cell offset of the map on the ramps, is measured. Section 10 of
the spec suspected that the covering element absorbs that offset, and
that the zero of v6 rests partly on that compensation. This confirms
it: without the diagonal neighbours the offset gets through.

Missions at 1.5 m: the 14 severe cells sit at two positions of the
`rough` zone, N +21.5 E -1.5 in 12 maps and N +17.5 E -0.5 in 2 maps:
truth slope 6.4 to 7.6 deg but truth step 0.52 to 0.53 m, map step 0.12
to 0.19 m, with 350 to 4800 observations per cell. The truth layer is
the cell mean (`tools/make_world.py`, truth = truth_mean), so this is
not a point sample read against a mean: the truth there changes by 0.3
to 0.5 m from one cell to the next, and the mission map smooths it to
about 0.1 m. At 1.2 m the same zone gives most of the additional cells
(26 maps at the first position, 7 other positions). No mission of that
campaign landed there: every final position lies on flat truth ground
(`2026-10-19_campaign_w1/README.md`).

## Decision

**The 1.5 m footprint stays in service.** The barrier of S8, zero
severe false accepts, is not negotiable. With 1.2 m it breaks on the
reference map (0 -> 2) and the severe cells on the mission maps triple
(14 -> 44), for 1 to 2 points of recall and no change of the minimum
zone.

What this says about the parameter, for the thesis: `footprint_m` is
not only the aircraft. Its measured need, a disc of 1.13 m, covers at
1 m resolution the centre cell and its four orthogonal neighbours. The
3x3 element of 1.5 m adds the four diagonal cells, and they are
measured here to be a margin the map error needs (P5 on the ramps). The
footprint derived from P4 is not the binding constraint; map error is.

Rejected: 1.2 m with an explicit safety margin of one cell. The margin
of `detect` dilates the refusal by whole cells before the footprint
test, so it refuses strictly more than the 3x3 element, for a second
parameter that says the same thing.

## Open, not fixed here

The barrier of S8 is not met on mission maps even at 1.5 m: 14 of 80
maps hold one severe cell, at two fixed cells of the `rough` zone where
the map smooths steps of 0.5 m to under 0.2 m. The zero holds on the
survey map v6, flown once. Cause not measured. It is a property of the
map, not of the footprint, and a change of its own.

    cd ~/ros2_ws/droneAfter
    python3 experiments/2026-10-20_footprint_p4/measure.py data_survey_v*.npz \
        --runs experiments/2026-10-19_campaign_w1/runs > measure.txt
    PYTHONPATH=experiments/2026-10-20_footprint_p4 \
        python3 experiments/2026-10-20_footprint_p4/inspect_severe.py \
        data_survey_v6.npz --runs experiments/2026-10-19_campaign_w1/runs

Files:

    measure.py           element, survey maps and mission maps, both footprints
    measure.txt          its output
    inspect_severe.py    position and geometry of every severe cell
    inspect_severe.txt   its output
