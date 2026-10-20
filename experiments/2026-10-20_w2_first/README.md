# First flights on W2, the world of rare landing zones, 20 Oct 2026

Built and flown while Dorian was away, on his authorisation of the W2
work. W2 is the second world of spec 5.x: "large, mostly not landable,
two or three safe zones far apart", to put C1 under tension.

## The world (`tools/make_world_w2.py`, a42a50c)

A 176 x 176 m corrugated field, strips 4 m wide sloping alternately up
and down at 20 deg (triangle wave, period 8 m, 1.46 m high), and three
flat pads of 16 x 16 m: the take-off pad at the origin, `pad_ne` at
E +48 N +40 and `pad_sw` at E -40 N -48, both 62.5 m from take-off and
124 m apart. Exploration area 120 x 120 m (`area_half_m` 60), map
200 x 200 m (`grid_size` 200); the field runs 28 m past the area, more
than the 18 m the lidar sees, so the flat ground plane beyond never
enters the map. Choices and the alternatives rejected are in the
generator's header. On the truth, with the detection chain: no slope
cell passes the cell test (smallest analysed slope 8.6 deg, step
0.36 m), each pad keeps 14 x 12 landable cells and holds a whole
footprint. `sim_session.sh` takes `WORLD=w2` and refuses a session
whose bridge reports another world.

## Reference flight: 150 kJ, alpha 0.5 (`runs/c1c2_a050_150kJ`)

| | |
|---|---|
| coverage | 100 % of the area, return when no frontier was left |
| energy | 97.3 kJ to the landing; 50 % coverage at 22.7 kJ, 90 % at 46.3 kJ, 99 % at 68.5 kJ (`energy_coverage.txt`) |
| registry | home pad confirmed at 21.9 s, `pad_ne` at 169.9 s (about 49 kJ in), `pad_sw` at 316.9 s; landed on `pad_sw` |
| map against truth (`map_check.py`) | recall 81.5 %, **0 false accepts**, marginal or severe |
| safety | 0 I1 violations, minimum margin 36 kJ, 0 failsafes |
| decision cycle | 96 ms median, 155 ms max on the 200 m map |

## Budget 48 kJ, one mission per configuration (exploratory)

48 kJ is about half of the energy of a full coverage, the ratio of the
16 kJ of W1 to its 32.4 kJ.

| configuration | coverage | min I1 margin | remote pad confirmed | landed |
|---|---|---|---|---|
| c1_c2, alpha 0 | 63.4 % | 1.55 kJ | none | home pad |
| c1_c2, alpha 0.5 | 69.5 % | 2.76 kJ | none | home pad |
| c1_c2, alpha 0.75 | 68.5 % | 1.02 kJ | none | home pad |
| c1_c2, alpha 1 | 54.9 % | 0.32 kJ | `pad_sw` at 108.8 s | home pad |
| B1 (gain, return at 30 %) | 62.0 % | 2.59 kJ | none | home pad |

0 I1 violations, 0 failsafes, 0 false accepts in all five. One mission
per configuration says nothing statistically; the order is the one of
the W1 gate F campaign (an interior optimum, pure gain last), which a
W2 campaign would have to confirm. At 48 kJ B1's fixed 30 % threshold
leaves it enough to come home from anywhere in the area, so it shows no
violation here; the tension is on coverage, not on safety.

## alpha changed live on W2 (`run_live.sh`, `runs/live_*`)

The test of `2026-10-20_alpha_live` on W2, at 48 kJ, the watcher running
`ros2 param set /exploration_manager alpha` 20 s after the first path.
W2 missions last about 150 s against 50 s on W1, so the change leaves
many choices after it:

| session | change | decisions after it | differing from the starting alpha | frontier paths before -> after | return energy of the choice before -> after |
|---|---|---|---|---|---|
| `live_a0to1_48kJ` | 0 -> 1 at 35.2 s | 8 | **7** | 15.7 to 20.5 m -> 16.6 to 89.2 m | 2.0 to 2.3 kJ -> 2.9 to 4.1 kJ |
| `live_a1to0_48kJ` | 1 -> 0 at 34.9 s | 12 | **12** | 18.5 to 54.1 m -> 15.3 to 36.9 m | 2.1 to 2.7 kJ -> 1.7 to 3.3 kJ |

Switched to 1 the aircraft leaves the take-off pad for far frontiers,
up to 49 m from the nearest confirmed zone; switched to 0 it goes back
to short hops of best margin (M = 1 at every choice). 0 I1 violations,
0 failsafes in both, landed on a confirmed zone. `alpha_live.png` in
each session plots the distance to the nearest zone against time with
the change drawn. This is the live demonstration Dorian asked for, and
W2 is the world that gives it room.

## Next

- A W2 campaign, the alphas and the baselines ten times each at 48 kJ,
  for gate F on W2 and figure F12 (B1 against B3 trajectories); about
  three hours, so on Dorian's go.

Files:

    run.sh             one W2 session: LABEL POLICY ALPHA BUDGET_KJ
    run_live.sh        one W2 session with a live change of alpha
    map_check.py       last map against the W2 truth, zones confirmed
    energy_coverage.py energy against coverage over a mission
    summarise.py       campaign metrics on the W2 truth
    runs/<label>/      setup.txt, logs, missions/, analysis.txt and the
                       outputs above
    MANIFEST.tsv       SHA-256 and size of every file
