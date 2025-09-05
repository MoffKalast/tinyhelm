import rospy

from tf2_ros import Buffer

from geometry_msgs.msg import TransformStamped, PoseStamped

def get_pose_in_frame(tf2_buffer: Buffer, parent_frame: str, target_frame: str):
    ts = tf2_buffer.lookup_transform(parent_frame, target_frame, rospy.Time(0))
    ps = PoseStamped()
    ps.header = ts.header
    ps.pose.position.x = ts.transform.translation.x
    ps.pose.position.y = ts.transform.translation.y
    ps.pose.position.z = ts.transform.translation.z
    ps.pose.orientation = ts.transform.rotation
    return ps