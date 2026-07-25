# behaviours.py
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Union
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from util import closest_pose_index, strip_repeated_poses

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

		bootstrap asks the helm to anchor the plan at the vessel before publishing it. The anchor is
		taken at publish time rather than here, because a RESTART re-publishes this same Intention
		and a position captured when the loiter began would be stale by then.
	"""
	name: str
	plan: Union[Path, PoseStamped]
	on_finish: StateAction
	bootstrap: bool = False

class Behaviours:

	@staticmethod
	def goal(robot_pose: PoseStamped, msg: PoseStamped) -> Intention:
		wrapper = Path()
		wrapper.header = msg.header
		wrapper.poses = [msg]
		return Intention(
			name="goal",
			plan=wrapper,
			on_finish=StateAction.HOLD_POSITION,
			bootstrap=True
		)

	@staticmethod
	def goal_and_return(robot_pose: PoseStamped, msg: PoseStamped) -> Intention:
		# The trailing pose is where we set off from, so unlike the leading anchor it is captured
		# now and never refreshed
		wrapper = Path()
		wrapper.header = msg.header
		wrapper.poses = [msg, robot_pose]
		return Intention(
			name="goal_and_return",
			plan=wrapper,
			on_finish=StateAction.HOLD_POSITION,
			bootstrap=True
		)

	@staticmethod
	def waypoints(robot_pose: PoseStamped, msg: Path) -> Intention:
		return Intention(
			name="waypoints",
			plan=msg,
			on_finish=StateAction.HOLD_POSITION,
			bootstrap=True
		)

	@staticmethod
	def waypoints_and_return(robot_pose: PoseStamped, msg: Path) -> Intention:
		rev = list(reversed(msg.poses))
		msg.poses.extend(rev[1:])  # skip the first reversed pose, it duplicates the turn waypoint
		return Intention(
			name="waypoints_and_return",
			plan=msg,
			on_finish=StateAction.HOLD_POSITION,
			bootstrap=True
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
			on_finish=StateAction.RESTART,
			bootstrap=True
		)

	@staticmethod
	def loiter_line(robot_pose: PoseStamped, msg: Path) -> Intention:
		idx = closest_pose_index(robot_pose, msg.poses)
		forward = msg.poses[idx:]
		backward = list(reversed(msg.poses))
		tail = msg.poses[0:idx]

		# The three runs meet at shared endpoints, so those joins have to be collapsed or the vessel
		# is handed legs of zero length that it reaches the instant it gets them
		msg.poses = strip_repeated_poses(forward + backward + tail)
		return Intention(
			name="loiter_line",
			plan=msg,
			on_finish=StateAction.RESTART,
			bootstrap=True
		)

	@staticmethod
	def stationkeeping(robot_pose: PoseStamped, msg: Optional[PoseStamped]) -> Intention:
		ps = msg if msg is not None else robot_pose
		return Intention(
			name="stationkeeping",
			plan=ps,
			on_finish=StateAction.IDLE
		)
