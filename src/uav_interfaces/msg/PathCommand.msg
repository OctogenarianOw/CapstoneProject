# Path for the trajectory executor, flown leg by leg on the calibrated
# rest-to-rest trapezoidal profile at cruise altitude. A new path replaces
# the one being flown at once (preemption, spec S4).
#
# A custom message rather than nav_msgs/Path: the executor needs an id to
# report arrival on, and only planar waypoints at one altitude.

std_msgs/Header header
uint32 id             # echoed in ExecutorStatus.path_id
float32[] north       # NED waypoints, metres; the first is the start
float32[] east
float32 altitude_m    # metres above the EKF origin
