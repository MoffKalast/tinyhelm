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

def drop_passed_legs(poses: List[PoseStamped], x: float, y: float) -> List[PoseStamped]:
    """Trims legs the vessel has already run past. Planning is not instant, so a path names a
    position the vessel held when the search started, and relaying it whole is what sends the vessel
    back to where it used to be. A leg counts as passed once the vessel's projection onto it reaches
    the far end. The anchor kept is the leg's own start rather than the vessel's position, so the
    controller still has a line to track instead of a point to chase."""
    if len(poses) < 2:
        return poses

    for i in range(len(poses) - 1):
        a, b = poses[i].pose.position, poses[i + 1].pose.position
        dx, dy = b.x - a.x, b.y - a.y
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-9:
            continue

        if ((x - a.x) * dx + (y - a.y) * dy) / length_sq < 1.0:
            return poses[i:]

    # Past the end of every leg, so the last one is all that is left to track
    return poses[-2:]

def closest_pose_index(robot_pose: PoseStamped, poses: List[PoseStamped]) -> int:
    """Find index of pose in `poses` closest to robot_pose."""
    rx, ry = robot_pose.pose.position.x, robot_pose.pose.position.y

    def distance_sq(p: PoseStamped):
        dx = p.pose.position.x - rx
        dy = p.pose.position.y - ry
        return dx * dx + dy * dy

    return min(range(len(poses)), key=lambda i: distance_sq(poses[i]))