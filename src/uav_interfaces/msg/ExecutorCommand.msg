# Mode command from the exploration manager to the trajectory executor.

uint8 TAKEOFF = 1     # arm, offboard, climb to altitude_m over the current point
uint8 HOLD = 2        # stop and hold the current position
uint8 LAND = 3        # land where the aircraft is (the manager brings it over a zone first)

std_msgs/Header header
uint8 command
float32 altitude_m    # TAKEOFF only, metres above the EKF origin
