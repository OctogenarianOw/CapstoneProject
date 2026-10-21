# Pit calibration, 21 Oct 2026 (W3, spec 5.1 and S9.5)

W3 needs a zone that the registry confirms from a distance and that a
closer observation invalidates while the guarantee may rest on it
(S9.5, gate B in flight, figure F13). The proposal agreed with Dorian on
21 Oct: a pad pierced by pits of 1 x 1 m, 4 m apart. The argument is
geometric: a ray flatter than depth / width does not reach a pit floor,
so at 8 m of altitude a pit 1 m wide and 1 m deep shows its floor only
within about 8 m, while the registry counts a pass within 14.4 m of the
touchdown point. Whether the mapping chain behaves that way is not
known, so the depth is measured here before W3 is built.

What is known before the flights, measured on the 80 last maps of
2026-10-20_campaign_w1_fix: around the two cells of the `rough` zone
that carry the severe false accepts, the map raises the low cells by
0.20 to 0.29 m and lowers the high ones by 0.11 to 0.13 m (medians),
so a true step of 0.53 m reads about 0.15 m. Those depressions are
0.4 m deep and 1 m wide, visible from about 20 m by the same geometry,
so their filling is not an occlusion effect; it says only that the map
fills depressions, whatever the cause.

## Set-up

`tools/make_world_pits.py`, world `pits`: PX4's flat ground and four
plates of 12 x 12 m in a row at east 40 m, north -36, -12, +12, +36,
0.5, 1.0, 1.5 and 2.0 m high, each pierced by four pits down to the
ground at cells 4 and 8 of 12 (east 38.5 and 42.5 m). Offline check
(`analyse_pits.py --truth`): the exact truth gives the full depth at
every pit and no plate confirmable; the truth with filled pits gives a
step of 0 and every plate confirmable. A first layout with 8 m plates
and pits one cell from the edge was dropped at that check: the 3 x 3
block read around 3 of the 4 pits held the edge step of the plate.

`pits_drive.py`: take-off at the origin, five north-south lines at
east 58.5, 54.5, 50.5, 46.5, 42.5 m, so the pit column at east 42.5 is
passed at 16, 12, 8, 4, 0 m and the one at 38.5 at 20, 16, 12, 8, 4 m,
far to near as a mission that confirms a pad then approaches it. Map
copied after each line (7 s hold, save period 5 s), position at 10 Hz.
Transits 22.5 m from the nearest pit, beyond the lidar's 18 m.

## Criterion, fixed before the flights

A depth works if every pit passed at 10 m or more has a step (S7, max
over the 3 x 3 block around the pit) under 0.24 m, the classifier
accepts it, and every pit passed within 5 m has a step over 0.48 m,
refused past the severe limit of S8. A block with an unobserved cell
counts as not hidden (I2 refuses it). Among the depths that work in
every pit of every flight, W3 takes the one hidden down to the smallest
distance, the smaller on a tie: the registry confirms after two passes
within 14.4 m of the touchdown point, and a pass at 14 m from the centre
of a 16 m pad comes within about 8.5 m of its nearest pits, so W3 needs
the widest band in which the trap is confirmed unseen. (First written
as "the smallest depth that works", revised before any flight for that
reason.) If no depth works, the deception by pits does not work in this
simulator and W3 is not built on it; that result is reported as it is.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-21_pit_calibration
    bash run.sh 3
    python3 analyse_pits.py runs/r*      # ROS and the workspace sourced

## Results

Three flights, r1 to r3, 3 of 3 at the first attempt, 0 failsafes
(`results.txt`, `pits.csv`). **No depth meets the criterion: no pit is
revealed, at any depth, even passed directly overhead.**

| depth | far readings hidden | pits passed within 5 m revealed | step / depth seen, passed within 2 m |
|---|---|---|---|
| 0.5 m | 17 of 30 | 0 of 18 | 0.18 / 0.17 m |
| 1.0 m | 20 of 30 | 0 of 18 | 0.19 / 0.18 m |
| 1.5 m | 17 of 30 | 0 of 18 | 0.14 / 0.13 m |
| 2.0 m | 17 of 30 | 0 of 18 | 0.14 / 0.12 m |

The far readings counted as not hidden are all blocks not yet OBSERVED
after lines 1 and 2 (49 of 49); no far reading shows a step of 0.24 m
or more. After the pass overhead (line 5) the plate still holds a whole
footprint in 10 of 12 plate flights, and in 3 of 3 for the 2.0 m
plate: a platform pierced by 2 m deep pits would be confirmed as a
landing zone. Read on the raw map of r1, the plate tops sit at their
true heights (1.96 to 2.01 m for 2.0 m, 0.96 to 1.00 m for 1.0 m) and
the pits show as dips of 0.09 to 0.21 m at the right cells.

Measured: the map fills 1 m pits whatever the range and the depth.
Not measured, the likely cause: the plate-top points around a pit that
the horizontal error of the pose (eph about 0.15 m) and of the attitude
put into its cell outnumber the near-vertical rays that reach its floor,
and the last oblique views after the pass weigh most in the filter.
This is the mechanism of P2, and it agrees with the filled depressions
of W1's rough zone (above).

Consequences:
- W3 is not built on 1 m pits (criterion above).
- A safety limit to declare in the thesis (spec 6.4): negative
  obstacles of one cell are not seen by this chain at 8 m. The zero
  severe false accepts of S8 holds on the geometry of W1, not on pitted
  ground.
- The questions left, wider pits and a hover over the pad, are taken
  by `2026-10-21_pit_calibration_2`.
