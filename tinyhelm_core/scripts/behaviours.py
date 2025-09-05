# behaviours.py
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Union
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from state_machine import HelmState

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
    state: HelmState
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
            state=HelmState.NAVIGATING,
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
            state=HelmState.NAVIGATING,
            on_finish=StateAction.HOLD_POSITION
        )

    @staticmethod
    def waypoints(robot_pose: PoseStamped, msg: Path) -> Intention:
        return Intention(
            name="waypoints",
            plan=msg,
            state=HelmState.NAVIGATING,
            on_finish=StateAction.HOLD_POSITION
        )

    @staticmethod
    def waypoints_and_return(robot_pose: PoseStamped, msg: Path) -> Intention:
        rev = list(reversed(msg.poses))
        msg.poses.extend(rev)
        return Intention(
            name="waypoints_and_return",
            plan=msg,
            state=HelmState.NAVIGATING,
            on_finish=StateAction.HOLD_POSITION
        )

    @staticmethod
    def loiter_circle(robot_pose: PoseStamped, msg: Path) -> Intention:
        if msg.poses:
            msg.poses.append(msg.poses[0])  # close the loop
        return Intention(
            name="loiter_circle",
            plan=msg,
            state=HelmState.NAVIGATING,
            on_finish=StateAction.RESTART
        )

    @staticmethod
    def loiter_line(robot_pose: PoseStamped, msg: Path) -> Intention:
        rev = list(reversed(msg.poses))
        msg.poses.extend(rev)
        return Intention(
            name="loiter_line",
            plan=msg,
            state=HelmState.NAVIGATING,
            on_finish=StateAction.RESTART
        )

    @staticmethod
    def stationkeeping(robot_pose: PoseStamped, msg: Optional[PoseStamped]) -> Intention:
        ps = msg if msg is not None else robot_pose
        return Intention(
            name="stationkeeping",
            plan=ps,
            state=HelmState.HOLD_POSITION,
            on_finish=StateAction.IDLE
        )
