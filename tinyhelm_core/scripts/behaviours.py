

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Behaviours:

    """
    Path generators, function names must match config behaviour_topic entires exactly.
    """

    @staticmethod
    def goal(robot_pose: PoseStamped, msg: PoseStamped):
        wrapper = Path()
        wrapper.header = msg.header
        wrapper.poses = [robot_pose, msg]
        return Path

    @staticmethod
    def goal_and_return(robot_pose: PoseStamped, msg: PoseStamped):
        wrapper = Path()
        wrapper.header = msg.header
        wrapper.poses = [robot_pose, msg, robot_pose]
        return Path

    @staticmethod
    def waypoints(robot_pose: PoseStamped, msg: Path):
        return msg

    @staticmethod
    def waypoints_and_return(robot_pose: PoseStamped, msg: Path):
        rev = list(reversed(msg.poses))
        msg.poses.extend(rev)
        return msg

    @staticmethod
    def loiter_circle(robot_pose: PoseStamped, msg: Path):
        msg.poses.append(msg.poses[0]) #close the loop
        return msg

    @staticmethod
    def loiter_line(robot_pose: PoseStamped, msg: Path):
        rev = list(reversed(msg.poses))
        msg.poses.extend(rev)
        return msg
    
    @staticmethod
    def stationkeeping(robot_pose: PoseStamped, msg: PoseStamped):
        if msg is None:
            return robot_pose
        return msg
