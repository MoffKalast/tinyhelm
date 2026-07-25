uint8 OK=0
uint8 GOAL_IN_OBSTACLE=1
uint8 GOAL_OUTSIDE_CORRIDOR=2
uint8 START_TRAPPED=3
uint8 NO_ROUTE=4
uint8 NO_COSTMAP=5

uint32 request_id
uint8 result

# Set when the vessel's own cell was inside the inflation and the search had to begin nearby, in
# which case path[0] is that nearby point and not the vessel
bool start_nudged

float64 plan_seconds
uint32 expansions

geometry_msgs/Point[] path
