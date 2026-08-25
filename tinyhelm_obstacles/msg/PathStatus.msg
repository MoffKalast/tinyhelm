# Reported whenever the watched route changes between clear and obstructed, and periodically either way, so the monitor learns about an intrusion without ever touching the grid itself. The steady report is also what it advances its progress and redraws its corridor on.
bool blocked

# Leg of the watched route the obstruction sits on, and how far along the route it is, so the monitor can tell an obstruction under the bow from one several waypoints away. -1 and 0 when clear.
int32 blocked_leg
float64 blocked_distance

float64 min_clearance
