# behaviours.py
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Union
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from util import closest_pose_index

class StateAction(Enum):
	HOLD_POSITION = 1
	RESTART = 2
	IDLE = 3
	RETURN_TO_HOME = 4

@dataclass
class Intention:
	"""
		What do we intend to do next? 
		A plan to execute in which state, and what happens after.
	"""
	name: str
	plan: Union[Path, PoseStamped]
	on_finish: StateAction

class Behaviours:

	@staticmethod
	def goal(robot_pose: PoseStamped, msg: PoseStamped) -> Intention:
		wrapper = Path()
		wrapper.header = msg.header
		wrapper.poses = [robot_pose, msg]
		return Intention(
			name="goal",
			plan=wrapper,
			on_finish=StateAction.HOLD_POSITION
		)

	@staticmethod
	def goal_and_return(robot_pose: PoseStamped, msg: PoseStamped) -> Intention:
		wrapper = Path()
		wrapper.header = msg.header
		wrapper.poses = [robot_pose, msg, robot_pose]
		return Intention(
			name="goal_and_return",
			plan=wrapper,
			on_finish=StateAction.HOLD_POSITION
		)

	@staticmethod
	def waypoints(robot_pose: PoseStamped, msg: Path) -> Intention:
		return Intention(
			name="waypoints",
			plan=msg,
			on_finish=StateAction.HOLD_POSITION
		)

	@staticmethod
	def waypoints_and_return(robot_pose: PoseStamped, msg: Path) -> Intention:
		rev = list(reversed(msg.poses))
		msg.poses.extend(rev[1:])  # skip the first reversed pose, it duplicates the turn waypoint
		return Intention(
			name="waypoints_and_return",
			plan=msg,
			on_finish=StateAction.HOLD_POSITION
		)

	@staticmethod
	def loiter_circle(robot_pose: PoseStamped, msg: Path) -> Intention:
		idx = closest_pose_index(robot_pose, msg.poses)
		shifted = msg.poses[idx:] + msg.poses[:idx]  # rotate so closest is first
		shifted.append(shifted[0])  # close loop
		msg.poses = shifted
		return Intention(
			name="loiter_circle",
			plan=msg,
			on_finish=StateAction.RESTART
		)

	@staticmethod
	def loiter_line(robot_pose: PoseStamped, msg: Path) -> Intention:
		idx = closest_pose_index(robot_pose, msg.poses)
		forward = msg.poses[idx:]
		backward = list(reversed(msg.poses))
		tail = msg.poses[0:idx]
		msg.poses = forward + backward + tail
		return Intention(
			name="loiter_line",
			plan=msg,
			on_finish=StateAction.RESTART
		)

	@staticmethod
	def stationkeeping(robot_pose: PoseStamped, msg: Optional[PoseStamped]) -> Intention:
		ps = msg if msg is not None else robot_pose
		return Intention(
			name="stationkeeping",
			plan=ps,
			on_finish=StateAction.IDLE
		)
