# The route the planner should re-examine against every costmap update. Clearance is carried with it
# so the trip threshold cannot drift out of step with what the route was planned against.
geometry_msgs/Point[] path
float64 clearance
