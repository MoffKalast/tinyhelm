# ControllerStatus.msg
# Reports when a controller status changes.

int8 status
string message   # optional human-readable description

int8 IDLE = 0
int8 ACTIVE = 1
int8 FINISHED = 2
int8 ESTOPPED  = 3
int8 PREEMPTED  = 4
int8 ABORTED  = 5
int8 ERROR  = 6
