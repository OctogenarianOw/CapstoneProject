# State of the trajectory executor, published at 10 Hz.

uint8 IDLE = 0        # on the ground, disarmed
uint8 TAKING_OFF = 1
uint8 HOLDING = 2     # at rest in the air, ready for a path
uint8 FOLLOWING = 3
uint8 LANDING = 4
uint8 LANDED = 5

std_msgs/Header header
uint8 state
uint32 path_id               # id of the path followed or last arrived at
bool arrived                 # path_id reached: real distance to its end, spec S4
float32 distance_to_goal_m   # horizontal, from the aircraft to the end of the path
uint32 leg                   # leg being flown, 0-based
uint32 legs
bool armed
bool offboard
