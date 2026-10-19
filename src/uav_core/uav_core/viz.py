"""
RViz messages for the live display of map, landing zones and frontiers.

Role: turn the manager's world model into standard ROS display messages,
so the map, the frontiers and the decisions can be watched in flight.

Frame: everything is published in 'map', ENU (x east, y north, z up), the
convention of RViz, converted from the NED of PX4 and of the grid:
x = east, y = north, z = -down. The launch file also publishes the
static transform map -> map_ned, so NED topics like /cloud_world display
in the same scene.

Layers, each its own topic so it can be toggled:
  /viz/elevation     PointCloud2, one point per seen cell, intensity =
                     height, colour by height in RViz
  /viz/observation   OccupancyGrid: OBSERVED 0, SPARSE 50, UNKNOWN -1;
                     the grid layout [i north, j east] row-major is the
                     OccupancyGrid layout (rows along y), no reshuffle
  /viz/landing       MarkerArray: candidate landing cells
  /viz/frontiers     MarkerArray: frontier cells, viewpoints, gain and
                     path distance; kept in cyan, rejected in grey,
                     refused by C1 in red, chosen viewpoint in blue
  /viz/zones         MarkerArray: registry zones, candidate yellow,
                     confirmed green, retired grey, take-off point blue
  /viz/path          Path: the path sent to the executor
  /viz/status        Marker: mission state, energy and I1 margin
  /viz/vehicle       Marker at the aircraft
Every MarkerArray starts with DELETEALL, so nothing stale survives a
cycle in which a frontier disappeared.
"""

import array

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from uav_core.observation import OBSERVED, SPARSE, UNKNOWN
from visualization_msgs.msg import Marker, MarkerArray

CYAN = (0.0, 0.8, 0.9, 1.0)
GREY = (0.55, 0.55, 0.55, 0.8)
GREEN = (0.1, 0.8, 0.2, 0.7)
BLUE = (0.2, 0.4, 1.0, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
RED = (0.9, 0.2, 0.2, 0.9)
YELLOW = (0.95, 0.8, 0.1, 0.8)
ZONE_COLOURS = {'CANDIDATE': YELLOW, 'CONFIRMED': GREEN, 'RETIRED': GREY}


def _color(rgba):
    return ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])


def ned_to_enu(north, east, down=0.0):
    """Return an ENU Point from NED coordinates."""
    return Point(x=float(east), y=float(north), z=float(-down))


def elevation_cloud(elevation, state, resolution, origin_ne, header):
    """One ENU point per seen cell, intensity = terrain height."""
    i, j = np.nonzero(state != UNKNOWN)
    north = origin_ne[0] + (i + 0.5) * resolution
    east = origin_ne[1] + (j + 0.5) * resolution
    height = -elevation[i, j].astype(np.float32)
    fields = [PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32,
                         count=1)
              for k, n in enumerate(('x', 'y', 'z', 'intensity'))]
    points = np.stack([east, north, height, height], axis=1).astype(
        np.float32)
    return point_cloud2.create_cloud(header, fields, points.tolist())


def observation_grid(state, resolution, origin_ne, header):
    """Return the observation states as an OccupancyGrid."""
    grid = OccupancyGrid()
    grid.header = header
    grid.info.resolution = float(resolution)
    grid.info.height, grid.info.width = (int(n) for n in state.shape)
    grid.info.origin.position.x = float(origin_ne[1])
    grid.info.origin.position.y = float(origin_ne[0])
    grid.info.origin.orientation.w = 1.0
    data = np.full(state.shape, -1, dtype=np.int8)
    data[state == OBSERVED] = 0
    data[state == SPARSE] = 50
    buf = array.array('b')
    buf.frombytes(np.ascontiguousarray(data).tobytes())
    grid.data = buf
    return grid


def _delete_all(header):
    m = Marker()
    m.header = header
    m.action = Marker.DELETEALL
    return m


def _cells_marker(header, ns, mid, cells, resolution, origin_ne, heights,
                  rgba, lift=0.3):
    m = Marker()
    m.header, m.ns, m.id = header, ns, mid
    m.type = Marker.CUBE_LIST
    m.scale.x = m.scale.y = 0.9 * resolution
    m.scale.z = 0.2
    m.color = _color(rgba)
    m.pose.orientation.w = 1.0
    for (i, j), h in zip(cells, heights):
        m.points.append(ned_to_enu(origin_ne[0] + (i + 0.5) * resolution,
                                   origin_ne[1] + (j + 0.5) * resolution,
                                   -(h + lift)))
    return m


def landing_markers(candidate, elevation, resolution, origin_ne, header):
    """Candidate landing cells as one green cube list."""
    out = MarkerArray()
    out.markers.append(_delete_all(header))
    i, j = np.nonzero(candidate)
    if i.size:
        heights = -elevation[i, j]
        out.markers.append(_cells_marker(header, 'landing', 0,
                                         zip(i, j), resolution, origin_ne,
                                         heights, GREEN))
    return out


def frontier_markers(catalogue, elevation, resolution, origin_ne, header,
                     altitude_m, chosen=None, inadmissible=()):
    """
    Frontier pieces, their viewpoints at cruise altitude and labels.

    Kept pieces are cyan, rejected ones grey, those C1 refused red, the
    viewpoint chosen blue.
    """
    out = MarkerArray()
    out.markers.append(_delete_all(header))
    mid = 0
    for f in catalogue.frontiers + catalogue.rejected:
        kept = not f.rejected
        rgba = CYAN if kept else GREY
        if kept and f.viewpoint_ij in inadmissible:
            rgba = RED
        heights = np.nan_to_num(-elevation[f.cells[:, 0], f.cells[:, 1]])
        out.markers.append(_cells_marker(header, 'frontier_cells', mid,
                                         map(tuple, f.cells), resolution,
                                         origin_ne, heights, rgba))
        if f.viewpoint_ne is not None:
            vp = Marker()
            vp.header, vp.ns, vp.id = header, 'viewpoints', mid
            vp.type = Marker.SPHERE
            vp.scale.x = vp.scale.y = vp.scale.z = 1.2
            is_chosen = chosen is not None and f.viewpoint_ij == chosen
            vp.color = _color(BLUE if is_chosen else rgba)
            vp.pose.position = ned_to_enu(*f.viewpoint_ne, -altitude_m)
            vp.pose.orientation.w = 1.0
            out.markers.append(vp)
            label = Marker()
            label.header, label.ns, label.id = header, 'labels', mid
            label.type = Marker.TEXT_VIEW_FACING
            label.scale.z = 1.0
            label.color = _color(WHITE)
            label.pose.position = ned_to_enu(*f.viewpoint_ne,
                                             -(altitude_m + 1.5))
            label.pose.orientation.w = 1.0
            label.text = (f'G {f.gain}  {f.path_dist_m:.0f} m'
                          if kept else f.rejected)
            out.markers.append(label)
        mid += 1
    return out


def zone_markers(zones, header):
    """Registry zones: a disc per zone at its touchdown point, by state."""
    out = MarkerArray()
    out.markers.append(_delete_all(header))
    for z in zones:
        if z.touchdown_ne is None:
            continue
        disc = Marker()
        disc.header, disc.ns, disc.id = header, 'zones', z.zone_id
        disc.type = Marker.CYLINDER
        radius = max(0.75, min(z.inscribed_radius_m, 5.0))
        disc.scale.x = disc.scale.y = 2.0 * radius
        disc.scale.z = 0.3
        disc.color = _color(BLUE if z.provisional
                            else ZONE_COLOURS.get(z.state, GREY))
        disc.pose.position = ned_to_enu(*z.touchdown_ne, -0.5)
        disc.pose.orientation.w = 1.0
        out.markers.append(disc)
        label = Marker()
        label.header, label.ns, label.id = header, 'zone_labels', z.zone_id
        label.type = Marker.TEXT_VIEW_FACING
        label.scale.z = 1.2
        label.color = _color(WHITE)
        label.pose.position = ned_to_enu(*z.touchdown_ne, -2.5)
        label.pose.orientation.w = 1.0
        kind = 'home' if z.provisional else z.state.lower()
        label.text = f'Z{z.zone_id} {kind} p{z.n_passes}'
        out.markers.append(label)
    return out


def path_message(waypoints_ne, altitude_m, header):
    """Return the planned path as a nav_msgs/Path in ENU."""
    path = Path()
    path.header = header
    for n, e in waypoints_ne:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = ned_to_enu(n, e, -altitude_m)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def status_marker(lines, north, east, header):
    """Return a text panel above the aircraft."""
    m = Marker()
    m.header, m.ns, m.id = header, 'status', 0
    m.type = Marker.TEXT_VIEW_FACING
    m.scale.z = 1.4
    m.color = _color(WHITE)
    m.pose.position = ned_to_enu(north, east, -16.0)
    m.pose.orientation.w = 1.0
    m.text = '\n'.join(lines)
    return m


def vehicle_marker(north, east, down, header):
    """Return a blue sphere at the aircraft."""
    m = Marker()
    m.header, m.ns, m.id = header, 'vehicle', 0
    m.type = Marker.SPHERE
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = _color(BLUE)
    m.pose.position = ned_to_enu(north, east, down)
    m.pose.orientation.w = 1.0
    return m
