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

To be filled after the flights.
