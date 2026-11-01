# rqt_graph during one mission, 1 Nov 2026

Question (Dorian): is Figure 3 of the report, the architecture, an
rqt_graph? It is not: it is drawn from the code by
`2026-10-24_thesis_figures/f1_architecture.py`. This directory shows the
real graph next to it and checks the figure against it.

`bash run.sh` flies one W1 mission (C1 + C2, alpha 0.75, 16 kJ, the
campaign parameters, no display) and, 5 s after the first frontier
transit starts, runs `capture_graph.py`, which builds the graph with
rqt_graph's own code and the window's default filters. Mission of
1 Nov 19:01: landed on zone 2, no failsafe. Output in `run/graph/`:

- `nodes_only.png`: what the rqt_graph window shows when opened.
- `nodes_topics_active.png`: nodes and topics, same filters.
- `nodes_topics_all.*`: every topic, 89 of them, mostly unused PX4 ones.
- `node_info.txt`, `topics.txt`: publishers and subscriptions per node
  and per topic, as `ros2 node info` and `ros2 topic info -v` give them.

What it shows:

- PX4 is not a ROS node. Its topics are published and read by the
  XRCE-DDS agent's participant (`_CREATED_BY_BARE_DDS_APP_`), so the
  default view has no PX4 at all, and the topic view keeps only
  `/fmu/out/vehicle_local_position_v1`, the one PX4 topic with three
  ROS subscribers; the others have a single ROS endpoint and fall under
  the default "hide leaf topics" filter. `/clock` is hidden by the quiet
  filter, `/viz/*` by "hide dead sinks" (no RViz running).
- rqt_graph cannot see inside `exploration_manager`: the registry, the
  energy model, C1 and C2 are Python modules it calls, not nodes.

Figure 3 against `node_info.txt`: every topic drawn is real and on the
right node. Three omissions:

1. `trajectory_executor` also subscribes to
   `/fmu/out/vehicle_local_position_v1` (arrival test on distance and
   speed, hold point, readiness to arm); the figure
   draws only vehicle_status and timesync_status into it.
2. `cloud_world` publishes `/cloud_world`, the point cloud for RViz.
3. `exploration_manager` subscribes to `/mission/stop`, the external stop
   used by the campaign scripts.

The first is a missing data flow; the other two are display and test
plumbing. The first was corrected in `f1_architecture.py` the same
evening (label under the PX4 to executor arrow); the caption of
Figure 3 names the other two.

`report_figure.sh` draws `rqt_graph_w1.png`, Appendix C of the report:
the topic view laid out top to bottom so that it stays readable at
16 cm.
