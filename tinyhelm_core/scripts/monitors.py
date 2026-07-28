# monitors.py
from enum import Enum

from tinyhelm_core.msg import MonitorStatus

MONITOR_STATUS_NAMES = {
	MonitorStatus.OK: "OK",
	MonitorStatus.REPLAN: "REPLAN",
	MonitorStatus.SLOW: "SLOW",
	MonitorStatus.HOLD: "HOLD",
	MonitorStatus.ESTOP: "ESTOP",
}

class MonitorAction(Enum):
	NOTHING = 1
	REVISE_PLAN = 2
	SLOW_DOWN = 3
	SUSPEND = 4
	STATIONKEEPING = 5

class Monitors:
	"""
		What the helm does when a monitor reports a status.
		Monitors never command anything themselves; they observe the active plan and propose,
		and the helm decides what to do with the proposal.

		The statuses are ordered by severity, so several monitors reduce to whichever of them is
		unhappiest and no precedence between them has to be spelled out here. REPLAN sits lowest
		above OK on purpose: it is a proposal attached to a mission that is otherwise going fine,
		and it must never outrank a monitor that wants the vessel stopped.
	"""

	ACTIONS = {
		MonitorStatus.OK: MonitorAction.NOTHING,
		MonitorStatus.REPLAN: MonitorAction.REVISE_PLAN,
		MonitorStatus.SLOW: MonitorAction.SLOW_DOWN,
		MonitorStatus.HOLD: MonitorAction.SUSPEND,
		MonitorStatus.ESTOP: MonitorAction.STATIONKEEPING,
	}

	@staticmethod
	def action_for(status: int) -> MonitorAction:
		return Monitors.ACTIONS.get(status, MonitorAction.NOTHING)
