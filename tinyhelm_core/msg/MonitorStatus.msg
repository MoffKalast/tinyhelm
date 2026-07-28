# MonitorStatus.msg
# Reports when a monitor status changes.

int8 status
string message   # optional human-readable description

# Ordered by severity. The helm remembers the last status from every monitor and acts on the worst
# of them, so anything inserted here has to keep that order meaningful.
int8 OK = 0
int8 REPLAN = 1
int8 SLOW = 2
int8 HOLD = 3
int8 ESTOP = 4
