"""
Exploration manager, sole holder of decision authority (spec S15).

Role: on every map, build the world model, decide where to fly and when
to land, and show the whole reasoning live. One component decides; the
frontier detector only produces a catalogue (spec 3.3), so C1 and C2 sit
in one place and can be switched off for the baselines.

Modes:
  observe  computes, displays and logs the world model, sends nothing;
           checks the perception chain on a deterministic flight
  explore  runs the mission state machine (mission_fsm.py) and drives
           the trajectory executor: autonomous exploration, no fixed
           trajectory

The decisions live in mission_fsm.MissionFSM, a pure class; this node
only moves messages, so every rule is tested without a simulator.

Cycle: one decision cycle per map received (1 Hz from cloud_world), so
each cycle works on a map no other cycle has seen, and the invariant is
checked at 1 Hz as S13 requires. A 10 Hz timer handles what needs no
map: take-off, the executor's landing, the vehicle marker.

Logged per cycle (S17): state, energy left, I1 margin, and every
candidate with E_go, E_return, E_total, admissibility, G, M and U, not
only the winner (S14). alpha can be changed in flight (S14), with the
slider of rqt_reconfigure (declared range 0 to 1, step 0.05) or:
    ros2 param set /exploration_manager alpha 0.25
Each change is logged as an event, so a figure can mark it.

Fault injection, for experiments only: inject_delay_s (default 0) holds
each decision that long after its inputs are read, as a slower computer
would. The first 190 m flight of 19 Oct broke I1 under C1 when its
decision step grew to 21.5 s; this parameter reproduces that latency on
W1, in a few minutes, on purpose and with a known value. It is logged
in config.json, so no mission can carry it unnoticed.
"""

from dataclasses import asdict
import os
import time

from nav_msgs.msg import OccupancyGrid, Path
from px4_msgs.msg import VehicleLocalPosition
from rcl_interfaces.msg import (FloatingPointRange, ParameterDescriptor,
                                SetParametersResult)
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Header
from uav_core import viz
from uav_core.energy_model import CALIBRATION_2026_10_19
from uav_core.frontier_detection import Blacklist
from uav_core.map_msg import msg_to_layers
from uav_core.mission_fsm import Inputs, MissionFSM, MissionParams
from uav_core.mission_log import MissionLog
from uav_core.slz_registry import SlzRegistry
from uav_core.world_model import build_world, THRESHOLDS, WorldParams
from uav_interfaces.msg import (ElevationMap, EnergyState, ExecutorCommand,
                                ExecutorStatus, PathCommand)
from visualization_msgs.msg import Marker, MarkerArray

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
MISSION_PARAMS = ('policy', 'alpha', 'reserve_fraction',
                  'return_threshold_fraction', 'dwell_s',
                  'bootstrap_cycles', 'energy_stale_s')


class ExplorationManager(Node):
    """World model, decisions, display and log."""

    def __init__(self):
        super().__init__('exploration_manager')
        world_defaults = asdict(WorldParams())
        mission_defaults = asdict(MissionParams())
        for name, value in world_defaults.items():
            self.declare_parameter(name, value)
        for name in MISSION_PARAMS:
            if name == 'alpha':
                # A declared range makes rqt_reconfigure show a slider.
                self.declare_parameter(name, mission_defaults[name],
                                       ParameterDescriptor(
                                           description='C2 weight: 1 '
                                           'information gain, 0 energy '
                                           'margin',
                                           floating_point_range=[
                                               FloatingPointRange(
                                                   from_value=0.0,
                                                   to_value=1.0,
                                                   step=0.05)]))
            else:
                self.declare_parameter(name, mission_defaults[name])
        self.declare_parameter('mode', 'observe')
        self.declare_parameter('log_root', '~/ros2_ws/droneAfter/missions')
        self.declare_parameter('repo', '~/ros2_ws/droneAfter')
        self.declare_parameter('inject_delay_s', 0.0)
        self.inject_delay_s = float(
            self.get_parameter('inject_delay_s').value)
        self.world_params = WorldParams(**{
            n: self.get_parameter(n).value for n in world_defaults})
        mission = {n: self.get_parameter(n).value for n in MISSION_PARAMS}
        mission['altitude_m'] = self.world_params.altitude_m
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('observe', 'explore'):
            raise SystemExit(f'unknown mode {self.mode}')

        self.model = CALIBRATION_2026_10_19
        self.blacklist = Blacklist()
        self.fsm = MissionFSM(MissionParams(**mission), self.model,
                              SlzRegistry(
                                  footprint_m=self.world_params.footprint_m,
                                  sensor_radius_m=(
                                      self.world_params.sensor_radius_m)),
                              self.blacklist)
        repo = os.path.expanduser(self.get_parameter('repo').value)
        self.log = MissionLog(self.get_parameter('log_root').value,
                              {'mode': self.mode,
                               'inject_delay_s': self.inject_delay_s,
                               **asdict(self.world_params), **mission,
                               'energy_model': asdict(self.model)},
                              repo=repo)
        self.fsm.log = self.log
        self.get_logger().info(f'mode {self.mode}, policy '
                               f'{mission["policy"]}, alpha '
                               f'{mission["alpha"]}, log in {self.log.dir}')

        self.vehicle = None
        self.pose_valid = False
        self.ex = None
        self.energy = None
        self.energy_rx = None
        self.stop = False
        self.last_path = None
        self.t0 = self.get_clock().now()

        self.create_subscription(ElevationMap, '/map/elevation',
                                 self.on_map, LATCHED)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position_v1',
                                 self.on_position, PX4_QOS)
        self.create_subscription(ExecutorStatus, '/executor/status',
                                 self.on_executor, 10)
        self.create_subscription(EnergyState, '/energy/state',
                                 self.on_energy, 10)
        self.create_subscription(Empty, '/mission/stop', self.on_stop, 10)
        self.cmd_pub = self.create_publisher(ExecutorCommand,
                                             '/executor/command', 10)
        self.path_pub = self.create_publisher(PathCommand,
                                              '/executor/path', 10)
        self.pub_cloud = self.create_publisher(PointCloud2,
                                               '/viz/elevation', 1)
        self.pub_class = self.create_publisher(PointCloud2,
                                               '/viz/classification', 1)
        self.pub_scores = {
            name: (self.create_publisher(PointCloud2, f'/viz/score/{name}',
                                         1), layer, THRESHOLDS[key])
            for name, layer, key in (
                ('slope', 'slope', 'max_slope_deg'),
                ('roughness', 'roughness', 'max_roughness_m'),
                ('step', 'step', 'max_step_m'))}
        self.pub_grid = self.create_publisher(OccupancyGrid,
                                              '/viz/observation', LATCHED)
        self.pub_landing = self.create_publisher(MarkerArray,
                                                 '/viz/landing', 1)
        self.pub_frontiers = self.create_publisher(MarkerArray,
                                                   '/viz/frontiers', 1)
        self.pub_zones = self.create_publisher(MarkerArray, '/viz/zones', 1)
        self.pub_path = self.create_publisher(Path, '/viz/path', LATCHED)
        self.pub_status = self.create_publisher(Marker, '/viz/status', 1)
        self.pub_vehicle = self.create_publisher(Marker, '/viz/vehicle', 1)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(0.1, self.on_timer)

    # ------------------------------------------------------------ inputs
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
        """Keep the latest NED position and its validity."""
        self.pose_valid = bool(msg.xy_valid and msg.z_valid)
        if self.pose_valid:
            self.vehicle = (msg.x, msg.y, msg.z)

    def on_executor(self, msg):
        """Keep the latest executor status."""
        self.ex = msg

    def on_energy(self, msg):
        """Keep the latest energy state and when it came."""
        self.energy = msg
        self.energy_rx = self.now_s()

    def on_stop(self, _msg):
        """External stop: return to a zone and land (S15.4)."""
        self.stop = True
        self.get_logger().info('stop requested')

    def on_params(self, params):
        """Apply alpha changes in flight (S14)."""
        for p in params:
            if p.name == 'alpha':
                self.fsm.params.alpha = float(p.value)
                self.log.event(self.now_s(), 'alpha', value=float(p.value))
        return SetParametersResult(successful=True)

    def inputs(self, world=None, res=1.0, origin=(0.0, 0.0)):
        """Gather one step's inputs for the state machine."""
        e = self.energy
        return Inputs(
            t=self.now_s(),
            vehicle_ne=self.vehicle[:2] if self.vehicle else None,
            pose_valid=self.pose_valid,
            ex_state=self.ex.state if self.ex else None,
            ex_path_id=self.ex.path_id if self.ex else 0,
            ex_arrived=self.ex.arrived if self.ex else False,
            remaining_j=e.remaining_j if e else None,
            capacity_j=e.capacity_j if e else None,
            energy_age_s=(self.now_s() - self.energy_rx
                          if self.energy_rx is not None else float('inf')),
            world=world, resolution=res, origin_ne=origin, stop=self.stop)

    # ------------------------------------------------------------ outputs
    def act(self, actions):
        """Turn state machine actions into executor messages."""
        for a in actions:
            if a.kind == 'takeoff':
                self.cmd_pub.publish(ExecutorCommand(
                    header=Header(), command=ExecutorCommand.TAKEOFF,
                    altitude_m=float(self.world_params.altitude_m)))
            elif a.kind == 'land':
                self.cmd_pub.publish(ExecutorCommand(
                    header=Header(), command=ExecutorCommand.LAND))
            elif a.kind == 'hold':
                self.cmd_pub.publish(ExecutorCommand(
                    header=Header(), command=ExecutorCommand.HOLD))
            elif a.kind == 'path':
                msg = PathCommand(header=Header(), id=a.path_id,
                                  altitude_m=float(
                                      self.world_params.altitude_m))
                msg.north = [float(p[0]) for p in a.waypoints_ne]
                msg.east = [float(p[1]) for p in a.waypoints_ne]
                self.path_pub.publish(msg)
                self.last_path = a.waypoints_ne
                self.pub_path.publish(viz.path_message(
                    a.waypoints_ne, self.world_params.altitude_m,
                    self.header()))
            self.log.event(self.now_s(), 'action', action=a.kind,
                           path_id=a.path_id, waypoints=a.waypoints_ne)

    def decide(self, inp):
        """Step the state machine, send its actions, stamp them sent."""
        actions = self.fsm.step(inp)
        self.act(actions)
        if actions:
            self.fsm.sent(self.now_s())

    def on_timer(self):
        """Run the steps that need no map, refresh the vehicle marker."""
        if self.vehicle is not None:
            self.pub_vehicle.publish(viz.vehicle_marker(*self.vehicle,
                                                        self.header()))
        if self.mode == 'explore':
            self.decide(self.inputs())

    def on_map(self, msg):
        """
        Run one decision cycle on a new map.

        The whole cycle is timed into the log, not only the world model:
        in the first 190 m flight of 19 Oct the logged time, world model
        alone, had a median of 179 ms while decisions took up to 18.7 s.
        """
        if self.vehicle is None:
            return
        t = self.now_s()
        t0 = time.perf_counter()
        elev, var, cnt, res, origin = msg_to_layers(msg)
        t1 = time.perf_counter()
        world = build_world(elev, var, cnt, res, origin, self.vehicle[:2],
                            self.world_params, self.blacklist, t)
        t2 = time.perf_counter()
        if self.mode == 'explore':
            inp = self.inputs(world, res, origin)
            if self.inject_delay_s > 0.0:
                time.sleep(self.inject_delay_s)
            self.decide(inp)
        t3 = time.perf_counter()
        self.display(world, elev, res, origin)
        t4 = time.perf_counter()
        world.timings_ms.update(decode=1e3 * (t1 - t0),
                                decision=1e3 * (t3 - t2),
                                display=1e3 * (t4 - t3))
        self.record(t, world, elev, var, cnt, res, origin)

    def display(self, world, elev, res, origin):
        """
        Publish the display layers somebody subscribes to.

        A layer unchecked in RViz has no subscriber, and is not built:
        on the 200 m grid each cloud holds up to 40 000 cells, and
        building all of them every cycle competed with the decisions.
        """
        h = self.header()
        d = self.fsm.last_decision
        refused = set()
        if d is not None:
            refused = {c.frontier.viewpoint_ij for c in d.candidates
                       if not c.admissible}
        chosen = (self.fsm.target.frontier.viewpoint_ij
                  if self.fsm.target is not None
                  and self.fsm.state == 'NAVIGATING' else None)

        def wanted(pub):
            return pub.get_subscription_count() > 0

        if wanted(self.pub_cloud):
            self.pub_cloud.publish(viz.elevation_cloud(
                elev, world.state, res, origin, h))
        if wanted(self.pub_class):
            self.pub_class.publish(viz.classification_cloud(
                elev, world.state, world.layers, world.landing, res, origin,
                h))
        for pub, layer, threshold in self.pub_scores.values():
            if wanted(pub):
                pub.publish(viz.score_cloud(
                    world.layers[layer], threshold, world.layers['valid'],
                    elev, res, origin, h))
        if wanted(self.pub_grid):
            self.pub_grid.publish(viz.observation_grid(world.state, res,
                                                       origin, h))
        if wanted(self.pub_landing):
            self.pub_landing.publish(viz.landing_markers(
                world.landing['candidate'], elev, res, origin, h))
        self.pub_frontiers.publish(viz.frontier_markers(
            world.catalogue, elev, res, origin, h,
            self.world_params.altitude_m, chosen, refused))
        self.pub_zones.publish(viz.zone_markers(
            list(self.fsm.registry.zones.values()), h))
        e = self.energy
        margin = self.fsm.last_margin
        lines = [f'{self.mode} | {self.fsm.state} | {self.fsm.params.policy}'
                 f' alpha {self.fsm.params.alpha:.2f}']
        if e is not None:
            lines.append(f'energy left {e.remaining_j / 1000:.1f} of '
                         f'{e.capacity_j / 1000:.0f} kJ')
        if margin is not None:
            lines.append(f'I1 margin {margin / 1000:.1f} kJ')
        lines.append(f'confirmed zones {len(self.fsm.registry.confirmed())}'
                     f' | violations {self.fsm.violations}')
        lines.append('green landing | light green margin | red refused '
                     '| grey uncertain')
        self.pub_status.publish(viz.status_marker(lines, *self.vehicle[:2],
                                                  h))

    def record(self, t, world, elev, var, cnt, res, origin):
        """Append the cycle to the mission log."""
        cat = world.catalogue
        d = self.fsm.last_decision
        candidates = []
        if d is not None:
            candidates = [{
                'viewpoint_ne': c.frontier.viewpoint_ne,
                'gain': c.frontier.gain, 'path_dist_m': c.frontier.path_dist_m,
                'e_go': c.e_go, 'e_return': c.e_return, 'e_total': c.e_total,
                'zone_id': c.zone_id, 'admissible': c.admissible,
                'g': c.g, 'm': c.m, 'u': c.u,
                'chosen': c is d.best} for c in d.candidates]
        e = self.energy
        self.log.cycle(t, {
            'mode': self.mode,
            'state': self.fsm.state,
            'vehicle_ned': self.vehicle,
            'remaining_j': e.remaining_j if e else None,
            'consumed_j': e.consumed_j if e else None,
            'i1_margin_j': self.fsm.last_margin,
            'violations': self.fsm.violations,
            'unknown_in_area': cat.unknown_in_area,
            'seen_in_area': int(((world.state != 0)
                                 & world.explore_mask).sum()),
            'landing_cells': int(world.landing['candidate'].sum()),
            'zones': [{'id': z.zone_id, 'state': z.state,
                       'provisional': z.provisional,
                       'touchdown_ne': z.touchdown_ne,
                       'area_m2': z.area_m2, 'passes': z.n_passes}
                      for z in self.fsm.registry.zones.values()],
            'frontiers': [{'cluster': f.cluster, 'size': f.size,
                           'viewpoint_ne': f.viewpoint_ne, 'gain': f.gain,
                           'path_dist_m': f.path_dist_m,
                           'rejected': f.rejected}
                          for f in cat.frontiers + cat.rejected],
            'candidates': candidates,
            'decision': d.reason if d is not None else None,
            'timings_ms': world.timings_ms})
        self.log.maybe_snapshot(t, elevation=elev, variance=var, count=cnt,
                                resolution=res, origin_north=origin[0],
                                origin_east=origin[1])
        self.get_logger().info(
            f'{t:6.1f} s {self.fsm.state}: {len(cat.frontiers)} frontiers, '
            f'{cat.unknown_in_area} unknown, margin '
            f'{self.fsm.last_margin}', throttle_duration_sec=5.0)


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
