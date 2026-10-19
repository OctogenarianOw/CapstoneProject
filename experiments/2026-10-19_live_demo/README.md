# Live demo with RViz and the Gazebo GUI, 19 Oct 2026

Session for Dorian to watch: sim_session.sh with RViz
(open_display.sh) and a Gazebo GUI client on the headless server
(open_sim_window.sh, software rendering through WSLg, llvmpipe), then
the exploration launch file, policy c1_c2, alpha 0.5, budget 20 kJ.
The wrappers exist because a first launch passed "rviz2" and "gz sim"
on the session command line, which the session's own cleanup matched:
it killed itself (documented in sim_session.sh).

Mission: take-off, 4 frontiers, none admissible at 47 s, return, landed
on the take-off zone 0.7 m from its point at 72 s; coverage 93.6 %,
0 I1 violations, minimum margin +0.09 kJ, a 1399 m2 zone confirmed.
Energy left at the end 1.5 kJ against a 2 kJ reserve: return and
landing cost about 0.6 kJ more than predicted, the landing taking
16.7 s against 14.3 s calibrated. The reserve covered it; the landing
phase in closed loop is to be measured on its own.

## Why the flat ground was not green (why_not_green.py, map_quality.py)

Dorian read the RViz elevation layer, coloured by height, as a safety
map; the classification was a thin marker layer under it. Fixed in the
display (/viz/classification). Beyond the display, on this map only
44.1 % of the truly flat cells were landing candidates:

| reason, flat zone | share |
|---|---|
| candidate | 44.1 % |
| footprint touches a refused cell | 15.8 % |
| roughness > 0.05 m | 14.8 % |
| slope > 6 deg | 10.6 % |
| never seen | 7.9 % |
| seen, not OBSERVED | 4.5 % |

Map error on the flat zone, three maps:

| map | RMS error | median observations | roughness > 0.05 m |
|---|---|---|---|
| this demo, 20 kJ, stopped by budget | 0.345 m | 310 | 28.6 % |
| first autonomous mission, 200 kJ, full coverage | 0.150 m | 1759 | 7.1 % |
| survey with the 2 m gate | 0.239 m | 737 | 6.9 % |

On the demo map, false roughness on flat ground falls with the number of
observations: 82 % under 50, 61 % at 50 to 100, 35 % at 100 to 200,
24 % at 200 to 500, 10 % above 500. A short budget leaves cells seen
rarely and from far, and the 3 x 3 analysis turns that noise into slope
and roughness. The refusals are false rejections, the safe direction.
The fused variance does not flag these cells (they pass the OBSERVED
gate), consistent with negative result 6.3 no. 1, errors correlated
within a scan: a count-based quality criterion is the candidate fix.

    python3 why_not_green.py missions/*/maps/<last>.npz
    python3 map_quality.py <snapshot> [<snapshot> ...]
