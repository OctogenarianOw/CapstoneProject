"""
Conversion between the elevation grid arrays and the ElevationMap message.

Role: one place that knows the row-major layout and the geometry fields,
so the publisher (cloud_world) and every consumer agree by construction.
NaN elevation and +inf variance mark cells never seen, as in the grid.

The arrays go through array.array buffers rather than numpy arrays or
lists: rosidl stores float32[] and int32[] as array.array, and a
byte copy avoids its per-element type check on 10 000 cells at 1 Hz.
"""

import array

import numpy as np
from uav_interfaces.msg import ElevationMap


def _buffer(values, dtype, typecode):
    out = array.array(typecode)
    out.frombytes(np.ascontiguousarray(values, dtype).tobytes())
    return out


def grid_to_msg(elevation, variance, count, resolution, origin_ne, header):
    """Pack the three layers and their geometry into a message."""
    msg = ElevationMap()
    msg.header = header
    msg.resolution = float(resolution)
    msg.origin_north = float(origin_ne[0])
    msg.origin_east = float(origin_ne[1])
    msg.rows, msg.cols = (int(n) for n in elevation.shape)
    msg.elevation = _buffer(elevation, np.float32, 'f')
    msg.variance = _buffer(variance, np.float32, 'f')
    msg.count = _buffer(count, np.int32, 'i')
    return msg


def msg_to_layers(msg):
    """Unpack a message into elevation, variance, count, res, origin."""
    shape = (msg.rows, msg.cols)
    elevation = np.frombuffer(msg.elevation, np.float32).reshape(shape)
    variance = np.frombuffer(msg.variance, np.float32).reshape(shape)
    count = np.frombuffer(msg.count, np.int32).reshape(shape)
    return (elevation.copy(), variance.copy(), count.copy(),
            float(msg.resolution),
            (float(msg.origin_north), float(msg.origin_east)))
