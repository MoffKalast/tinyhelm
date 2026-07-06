# MonitorStatus.msg
# Reports when a monitor status changes.

int8 status
string message   # optional human-readable description

int8 OK = 0
int8 WARN = 1
int8 REPLAN  = 2
int8 ESTOP  = 3
int8 INTERNAL_ERROR = 4
int8 OBSERVED_ERROR = 5