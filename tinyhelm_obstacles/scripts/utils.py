import math

import rospy
import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def robot_position(buffer, planning_frame, robot_frame):
	try:
		tf = buffer.lookup_transform(planning_frame, robot_frame, rospy.Time(0))
		return tf.transform.translation.x, tf.transform.translation.y
	except tf2_ros.TransformException as e:
		rospy.logwarn_throttle(5.0, "no %s -> %s: %s" % (planning_frame, robot_frame, e))
		return None

def path_to_planning_frame(buffer, msg, planning_frame):
	"""Returns the poses in the planning frame, or None when a stated frame cannot be resolved.
	Everything downstream assumes the planning frame, and untransformed coordinates would have us
	monitoring somewhere else entirely without saying so."""
	frame = msg.header.frame_id
	if not frame or frame == planning_frame or not msg.poses:
		return list(msg.poses)

	try:
		tf = buffer.lookup_transform(planning_frame, frame, rospy.Time(0))
	except tf2_ros.TransformException as e:
		rospy.logwarn("path in '%s' could not be transformed, ignoring it: %s" % (frame, e))
		return None

	return [tf2_geometry_msgs.do_transform_pose(pose, tf) for pose in msg.poses]

def poses_to_xy(poses):
	return [(p.pose.position.x, p.pose.position.y) for p in poses]

def poses_to_xyz(poses):
	return [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]

def make_pose(frame, x, y, z):
	pose = PoseStamped()
	pose.header.frame_id = frame
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = z
	pose.pose.orientation.w = 1.0
	return pose

def make_path(frame, points):
	msg = Path()
	msg.header.frame_id = frame
	msg.header.stamp = rospy.Time.now()
	msg.poses = [make_pose(frame, x, y, z) for x, y, z in points]
	return msg

def segment_distance(px, py, ax, ay, bx, by):
	dx = bx - ax
	dy = by - ay
	len2 = dx * dx + dy * dy
	t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2)) if len2 > 0.0 else 0.0
	return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def match_index(mission, plan, tolerance):
	"""Index of the first mission waypoint the controller still has ahead of it, or None when the
	plan carries none of them.

	plan[0] is the anchor of the leg in progress and is therefore already behind us, so it is
	excluded. Corrections end exactly on mission waypoints, which is what makes this work; their
	detour points match nothing and are ignored."""
	if len(plan) < 2 or not mission:
		return None

	ahead = plan[1:]
	for index, (mx, my, _) in enumerate(mission):
		for px, py in ahead:
			if math.hypot(px - mx, py - my) <= tolerance:
				return index

	return None

class PendingRequest:
	"""Bookkeeping for one outstanding planner request. Holds no publisher and calls nothing back:
	the monitor owns the timer and does the publishing."""

	def __init__(self, timeout, retries):
		self.timeout = timeout
		self.retries = retries
		self.request_id = None
		self.mission_index = None
		self.attempts = 0
		self.deadline = rospy.Time(0)

	def open(self, request_id, mission_index):
		self.request_id = request_id
		self.mission_index = mission_index
		self.attempts = 1
		self.deadline = rospy.Time.now() + rospy.Duration(self.timeout)

	def outstanding(self):
		return self.request_id is not None

	def matches(self, request_id):
		return self.request_id is not None and request_id == self.request_id

	def expired(self, now):
		return self.request_id is not None and now >= self.deadline

	def exhausted(self):
		return self.attempts > self.retries

	def retry(self):
		self.attempts += 1
		self.deadline = rospy.Time.now() + rospy.Duration(self.timeout)

	def close(self):
		self.request_id = None
		self.mission_index = None
		self.attempts = 0
