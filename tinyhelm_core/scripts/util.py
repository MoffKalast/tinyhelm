import math
import rospy

from tf2_ros import Buffer

from typing import List

from geometry_msgs.msg import TransformStamped, PoseStamped

def get_pose_in_frame(tf2_buffer: Buffer, parent_frame: str, target_frame: str):
    """Does a lookup and casts to PoseStamped"""
    ts = tf2_buffer.lookup_transform(parent_frame, target_frame, rospy.Time(0))
    ps = PoseStamped()
    ps.header = ts.header
    ps.pose.position.x = ts.transform.translation.x
    ps.pose.position.y = ts.transform.translation.y
    ps.pose.position.z = ts.transform.translation.z
    ps.pose.orientation = ts.transform.rotation
    return ps

def strip_repeated_poses(poses: List[PoseStamped], epsilon: float = 1e-3) -> List[PoseStamped]:
    """Drops poses that repeat the one before them. Out and back routes join their two traversals
    at a shared endpoint, which would otherwise leave a zero length leg the controller reaches the
    instant it is handed one."""
    out = []
    for pose in poses:
        if out:
            previous, current = out[-1].pose.position, pose.pose.position
            if math.hypot(current.x - previous.x, current.y - previous.y) <= epsilon and abs(current.z - previous.z) <= epsilon:
                continue
        out.append(pose)

    return out

def closest_pose_index(robot_pose: PoseStamped, poses: List[PoseStamped]) -> int:
    """Find index of pose in `poses` closest to robot_pose."""
    rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y

    def distance_sq(p: PoseStamped):
        dx = p.pose.position.x - rx
        dy = p.pose.position.y - ry
        return dx * dx + dy * dy

    return min(range(len(poses)), key=lambda i: distance_sq(poses[i]))