# monitors.py
from enum import Enum

from tinyhelm_core.msg import MonitorStatus

MONITOR_STATUS_NAMES = {
	MonitorStatus.OK: "OK",
	MonitorStatus.WARN: "WARN",
	MonitorStatus.REPLAN: "REPLAN",
	MonitorStatus.ESTOP: "ESTOP",
	MonitorStatus.INTERNAL_ERROR: "INTERNAL_ERROR",
	MonitorStatus.OBSERVED_ERROR: "OBSERVED_ERROR",
}

class MonitorAction(Enum):
	NOTHING = 1
	REVISE_PLAN = 2
	STATIONKEEPING = 3

class Monitors:
	"""
		What the helm does when a monitor reports a status.
		Monitors never command anything themselves; they observe the active plan and propose,
		and the helm decides what to do with the proposal.
	"""

	ACTIONS = {
		MonitorStatus.OK: MonitorAction.NOTHING,
		MonitorStatus.WARN: MonitorAction.NOTHING,
		MonitorStatus.REPLAN: MonitorAction.REVISE_PLAN,
		MonitorStatus.ESTOP: MonitorAction.STATIONKEEPING,
		MonitorStatus.INTERNAL_ERROR: MonitorAction.NOTHING,
		MonitorStatus.OBSERVED_ERROR: MonitorAction.NOTHING,
	}

	@staticmethod
	def action_for(status: int) -> MonitorAction:
		return Monitors.ACTIONS.get(status, MonitorAction.NOTHING)
