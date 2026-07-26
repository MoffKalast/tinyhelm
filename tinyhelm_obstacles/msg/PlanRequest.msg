# The planner holds no mission state: everything it needs to answer arrives here, so a stale request cannot make it plan against a corridor that has since moved.
uint32 request_id
geometry_msgs/Point start
geometry_msgs/Point goal

# Hard clearance the route must keep from any obstacle. The distance over which the soft standoff penalty falls away is not sent: both ends read soft_radius from the same parameters.
float64 clearance

# The route may not leave a tube of corridor_radius around this polyline. 
# Sent as the polyline rather than as capsules so it stays readable on the wire and both ends derive the same tube from it.
geometry_msgs/Point[] corridor
float64 corridor_radius
