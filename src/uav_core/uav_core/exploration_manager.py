"""
Exploration manager, sole holder of decision authority (spec S15).

Role: on every map, build the world model, decide where to fly and when
to land, and show the whole reasoning live. One component decides; the
frontier detector only produces a catalogue (spec 3.3), so C1 and C2 sit
in one place and can be switched off for the baselines.

This version runs in mode 'observe' only: it computes and displays the
world model and logs every cycle, but sends no command. It lets the
perception chain be checked live on a deterministic flight (the survey)
before it is given authority over the aircraft. The decision modes come
with the landing zone registry, C1, C2, the energy monitor and the
trajectory executor.

Cycle: one per map received (1 Hz from cloud_world), never on a timer of
its own, so each cycle works on a map no other cycle has seen. The
vehicle marker is refreshed at 5 Hz from the PX4 position.
"""

from dataclasses import asdict
import os

from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from uav_core import viz
from uav_core.frontier_detection import Blacklist
from uav_core.map_msg import msg_to_layers
from uav_core.mission_log import MissionLog
from uav_core.world_model import build_world, WorldParams
from uav_interfaces.msg import ElevationMap
from visualization_msgs.msg import Marker, MarkerArray

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


class ExplorationManager(Node):
    """World model, display and log; decisions to come."""

    def __init__(self):
        super().__init__('exploration_manager')
        defaults = WorldParams()
        for name, value in asdict(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('mode', 'observe')
        self.declare_parameter('log_root', '~/ros2_ws/droneAfter/missions')
        self.declare_parameter('repo', '~/ros2_ws/droneAfter')
        self.params = WorldParams(**{
            name: self.get_parameter(name).value
            for name in asdict(defaults)})
        self.mode = self.get_parameter('mode').value
        if self.mode != 'observe':
            raise SystemExit(f'mode {self.mode} not implemented yet')

        repo = os.path.expanduser(self.get_parameter('repo').value)
        self.log = MissionLog(self.get_parameter('log_root').value,
                              {'mode': self.mode, **asdict(self.params)},
                              repo=repo)
        self.get_logger().info(f'mission log in {self.log.dir}')

        self.blacklist = Blacklist()
        self.vehicle = None
        self.t0 = self.get_clock().now()

        self.create_subscription(ElevationMap, '/map/elevation',
                                 self.on_map, LATCHED)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, PX4_QOS)
        self.pub_cloud = self.create_publisher(PointCloud2,
                                               '/viz/elevation', 1)
        self.pub_grid = self.create_publisher(OccupancyGrid,
                                              '/viz/observation', LATCHED)
        self.pub_landing = self.create_publisher(MarkerArray,
                                                 '/viz/landing', 1)
        self.pub_frontiers = self.create_publisher(MarkerArray,
                                                   '/viz/frontiers', 1)
        self.pub_vehicle = self.create_publisher(Marker, '/viz/vehicle', 1)
        self.create_timer(0.2, self.publish_vehicle)

    def now_s(self):
        """Seconds since the node started."""
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def header(self):
        """Header in the ENU display frame, stamped now."""
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'map'
        return h

    def on_position(self, msg):
        """Keep the latest NED position if PX4 says it is valid."""
        if msg.xy_valid and msg.z_valid:
            self.vehicle = (msg.x, msg.y, msg.z)

    def publish_vehicle(self):
        """Refresh the aircraft marker."""
        if self.vehicle is not None:
            self.pub_vehicle.publish(viz.vehicle_marker(*self.vehicle,
                                                        self.header()))

    def on_map(self, msg):
        """Run one cycle on a new map."""
        if self.vehicle is None:
            return
        t = self.now_s()
        elev, var, cnt, res, origin = msg_to_layers(msg)
        vehicle_ne = self.vehicle[:2]
        world = build_world(elev, var, cnt, res, origin, vehicle_ne,
                            self.params, self.blacklist, t)
        h = self.header()
        self.pub_cloud.publish(viz.elevation_cloud(elev, world.state, res,
                                                   origin, h))
        self.pub_grid.publish(viz.observation_grid(world.state, res,
                                                   origin, h))
        self.pub_landing.publish(viz.landing_markers(
            world.landing['candidate'], elev, res, origin, h))
        self.pub_frontiers.publish(viz.frontier_markers(
            world.catalogue, elev, res, origin, h, self.params.altitude_m))

        cat = world.catalogue
        self.log.cycle(t, {
            'mode': self.mode,
            'vehicle_ned': self.vehicle,
            'unknown_in_area': cat.unknown_in_area,
            'seen_in_area': int(((world.state != 0)
                                 & world.explore_mask).sum()),
            'landing_cells': int(world.landing['candidate'].sum()),
            'landing_zones': len(world.landing['zones']),
            'frontiers': [{'cluster': f.cluster, 'size': f.size,
                           'viewpoint_ne': f.viewpoint_ne, 'gain': f.gain,
                           'path_dist_m': f.path_dist_m,
                           'rejected': f.rejected}
                          for f in cat.frontiers + cat.rejected],
            'timings_ms': world.timings_ms})
        self.log.maybe_snapshot(t, elevation=elev, variance=var, count=cnt,
                                resolution=res, origin_north=origin[0],
                                origin_east=origin[1])
        total = sum(world.timings_ms.values())
        self.get_logger().info(
            f'cycle {t:6.1f} s: {len(cat.frontiers)} frontiers kept, '
            f'{len(cat.rejected)} rejected, {cat.unknown_in_area} unknown '
            f'in area, {int(world.landing["candidate"].sum())} landing '
            f'cells, {total:.0f} ms', throttle_duration_sec=5.0)


def main(args=None):
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = ExplorationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.log.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
