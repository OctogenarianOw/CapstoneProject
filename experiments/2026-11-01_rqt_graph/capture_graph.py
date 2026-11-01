"""
rqt_graph of the running system, captured during one W1 mission, to
check the architecture figure of the report (Figure 3) against what ROS
itself reports.

Role: build the graph exactly as the rqt_graph window does (its own
rosgraph2_impl.Graph and RosGraphDotcodeGenerator, with the window's
default filters), in its three views, write each as DOT, PNG and SVG,
and list every node's publishers and subscriptions, as `ros2 node info`
would, plus the endpoints of every topic.

Choice: call rqt_graph's code from a script rather than screenshot the
window: same graph, same layout engine (graphviz dot), at print
resolution and without a display. Rejected: a desktop capture of
rqt_graph under WSLg, whose size and zoom depend on the window.

    python3 capture_graph.py OUT_DIR    (ROS sourced, during a mission)
"""
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from qt_dotgraph.pydotfactory import PydotFactory
from rqt_graph.dotcode import (NODE_NODE_GRAPH, NODE_TOPIC_ALL_GRAPH,
                               NODE_TOPIC_GRAPH, RosGraphDotcodeGenerator)
from rqt_graph.rosgraph2_impl import Graph

# The defaults of the rqt_graph window (ros_graph.py, restore_settings).
DEFAULTS = dict(ns_filter='/', topic_filter='/',
                hide_single_connection_topics=True,
                hide_dead_end_topics=True, cluster_namespaces_level=2,
                accumulate_actions=True, orientation='LR', quiet=True,
                unreachable=True, group_tf_nodes=True, hide_tf_nodes=False,
                group_image_nodes=True, hide_dynamic_reconfigure=True)
VIEWS = {'nodes_only': NODE_NODE_GRAPH,              # the window's default
         'nodes_topics_active': NODE_TOPIC_GRAPH,
         'nodes_topics_all': NODE_TOPIC_ALL_GRAPH}


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    # Named like the window's own node, which its quiet filter hides.
    node = rclpy.create_node('rqt_graph_capture')
    t_end = time.time() + 4.0            # let discovery settle
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    graph = Graph(node)
    graph.set_node_stale(5.0)
    graph.update()
    gen = RosGraphDotcodeGenerator(node)
    for name, mode in VIEWS.items():
        dot = gen.generate_dotcode(rosgraphinst=graph, graph_mode=mode,
                                   dotcode_factory=PydotFactory(),
                                   **DEFAULTS)
        path = out / f'{name}.dot'
        path.write_text(dot)
        for fmt, extra in (('png', ['-Gdpi=170']), ('svg', [])):
            subprocess.run(['dot', f'-T{fmt}', *extra, str(path), '-o',
                            str(out / f'{name}.{fmt}')], check=True)
    lines = []
    for name, ns in sorted(node.get_node_names_and_namespaces()):
        full = (ns.rstrip('/') + '/' + name)
        lines.append(full)
        for label, fn in (('publishes', node.get_publisher_names_and_types_by_node),
                          ('subscribes', node.get_subscriber_names_and_types_by_node)):
            for topic, types in sorted(fn(name, ns)):
                lines.append(f'  {label:10s} {topic}  [{", ".join(types)}]')
    (out / 'node_info.txt').write_text('\n'.join(lines) + '\n')
    lines = []
    for topic, types in sorted(node.get_topic_names_and_types()):
        pubs = sorted({p.node_name for p in node.get_publishers_info_by_topic(topic)})
        subs = sorted({s.node_name for s in node.get_subscriptions_info_by_topic(topic)})
        lines.append(f'{topic}  [{", ".join(types)}]\n'
                     f'  publishers  {", ".join(pubs) or "none"}\n'
                     f'  subscribers {", ".join(subs) or "none"}')
    (out / 'topics.txt').write_text('\n'.join(lines) + '\n')
    print('graph written to', out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
