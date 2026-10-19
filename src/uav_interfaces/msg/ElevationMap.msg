# Fused 2.5D elevation grid, published by cloud_world (spec S5).
#
# Cell (i, j) covers NED north [origin_north + i res, origin_north + (i + 1) res)
# and east [origin_east + j res, origin_east + (j + 1) res). Arrays are
# row-major, index i * cols + j, i along north and j along east.
#
# A custom message rather than three Float32MultiArray: the layers only
# make sense together with their geometry, and a typed message carries
# both without conventions hidden in a layout label.

std_msgs/Header header          # frame_id map_ned, stamp of publication

float32 resolution              # metres per cell
float32 origin_north            # NED north of the corner of cell (0, 0), metres
float32 origin_east             # NED east of the corner of cell (0, 0), metres
uint32 rows                     # cells along north
uint32 cols                     # cells along east

float32[] elevation             # NED z of the terrain, metres, down; NaN if never seen
float32[] variance              # fused variance, m^2; +inf if never seen
int32[] count                   # observations fused into the cell
