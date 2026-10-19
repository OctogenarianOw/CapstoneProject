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
  /viz/classification PointCloud2 at terrain height, one coloured box per
                     seen cell: green landing candidate, light green
                     sound cell too close to a refused one for the
                     footprint, red refused by slope, roughness or step,
                     grey not observed well enough to judge. Shown by
                     default: on 19 Oct the height colours of the
                     elevation layer were read as a safety map
  /viz/elevation     PointCloud2, one point per seen cell, intensity =
                     height, colour by height in RViz (off by default)
  /viz/score/slope, /viz/score/roughness, /viz/score/step
                     PointCloud2 at terrain height, each analysed cell
                     coloured by its value over the acceptance threshold:
                     green at half the threshold or less, yellow at the
                     threshold, red at twice and more. Shows which
                     criterion refuses a cell (off by default)
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
from sensor_msgs.msg import PointCloud2, PointField
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
    height = np.nan_to_num(-elevation[i, j]).astype(np.float32)
    cloud = np.zeros(len(i), dtype=[('x', '<f4'), ('y', '<f4'),
                                    ('z', '<f4'), ('intensity', '<f4')])
    cloud['x'], cloud['y'] = east, north
    cloud['z'], cloud['intensity'] = height, height
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(i)
    msg.fields = [PointField(name=n, offset=4 * k,
                             datatype=PointField.FLOAT32, count=1)
                  for k, n in enumerate(('x', 'y', 'z', 'intensity'))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(i)
    msg.is_dense = True
    msg.data = cloud.tobytes()
    return msg


CLASS_RGB = {
    'candidate': (25, 200, 50),
    'margin': (170, 230, 110),
    'refused': (215, 50, 50),
    'uncertain': (140, 140, 140),
}


def _packed_cloud(header, east, north, up, rgb):
    """PointCloud2 with x, y, z and a packed rgb field, built in bulk."""
    cloud = np.zeros(len(east), dtype=[('x', '<f4'), ('y', '<f4'),
                                       ('z', '<f4'), ('rgb', '<u4')])
    cloud['x'], cloud['y'], cloud['z'] = east, north, up
    cloud['rgb'] = rgb
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(east)
    msg.fields = [PointField(name=n, offset=4 * k,
                             datatype=PointField.FLOAT32, count=1)
                  for k, n in enumerate(('x', 'y', 'z', 'rgb'))]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(east)
    msg.is_dense = True
    msg.data = cloud.tobytes()
    return msg


def classification_cloud(elevation, state, layers, landing, resolution,
                         origin_ne, header):
    """One box per seen cell, coloured by what the detector made of it."""
    seen = state != UNKNOWN
    uncertain = seen & ((state != OBSERVED) | ~layers['valid'])
    cell_ok = landing['cell_ok']
    candidate = landing['candidate']
    refused = seen & ~uncertain & ~cell_ok
    margin = cell_ok & ~candidate
    rgb = np.zeros(state.shape, dtype=np.uint32)
    for mask, name in ((uncertain, 'uncertain'), (refused, 'refused'),
                       (margin, 'margin'), (candidate, 'candidate')):
        r, g, b = CLASS_RGB[name]
        rgb[mask] = (r << 16) | (g << 8) | b
    i, j = np.nonzero(seen)
    north = origin_ne[0] + (i + 0.5) * resolution
    east = origin_ne[1] + (j + 0.5) * resolution
    up = np.nan_to_num(-elevation[i, j])
    return _packed_cloud(header, east, north, up, rgb[i, j])


def ratio_colour(ratio):
    """Pack green -> yellow -> red for value / threshold in [0.5, 2]."""
    x = np.clip((np.asarray(ratio, dtype=np.float64) - 0.5) / 1.5, 0.0, 1.0)
    # 0 green (0.5 x threshold), 1/3 yellow (threshold), 1 red (2 x).
    r = np.where(x < 1 / 3, x * 3, 1.0)
    g = np.where(x < 1 / 3, 1.0, 1.0 - (x - 1 / 3) * 1.5)
    r8 = (r * 230).astype(np.uint32)
    g8 = (g * 210).astype(np.uint32)
    return (r8 << 16) | (g8 << 8) | np.uint32(40)


def score_cloud(values, threshold, valid, elevation, resolution, origin_ne,
                header):
    """One box per analysed cell, coloured by value over threshold."""
    i, j = np.nonzero(valid & np.isfinite(values))
    north = origin_ne[0] + (i + 0.5) * resolution
    east = origin_ne[1] + (j + 0.5) * resolution
    up = np.nan_to_num(-elevation[i, j])
    return _packed_cloud(header, east, north, up,
                         ratio_colour(values[i, j] / threshold))


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
