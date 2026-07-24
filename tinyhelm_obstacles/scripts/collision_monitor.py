#!/usr/bin/env python3
import rospy
import tf2_ros

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist
from tinyhelm_core.msg import MonitorStatus


class CollisionMonitor:
	"""Dumb and fast by design: watches raw obstacle detections (no decay, no hit
	counting) and screams EMERGENCY if anything enters a speed-scaled stop zone ahead
	of the vessel. All reaction policy lives in the helm core."""

	def __init__(self):
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")
		self.halfwidth = rospy.get_param("~vessel_halfwidth", 1.0)
		self.margin = rospy.get_param("~margin", 2.0)
		self.horizon = rospy.get_param("~braking_horizon", 3.0)
		self.min_points = rospy.get_param("~min_points", 2)
		self.heartbeat_period = rospy.get_param("~heartbeat_period", 1.0)

		self.speed = 0.0
		self.last_emergency = rospy.Time(0)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		rospy.Subscriber("/obstacle_cloud", PointCloud2, self.cloud_callback, queue_size=5)
		rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=5)
		self.status_pub = rospy.Publisher("/tinyhelm/monitor/collision", MonitorStatus, queue_size=5)

		rospy.loginfo("collision_monitor: halfwidth %.1fm, margin %.1fm, horizon %.1fs", self.halfwidth, self.margin, self.horizon)

	def spin(self):
		"""Heartbeat loop in the main thread instead of rospy.Timer, so a sim time jump
		backwards (rosbag reset) resets cleanly instead of stalling the timer."""
		rate = rospy.Rate(1.0 / self.heartbeat_period)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSTimeMovedBackwardsException:
				rospy.logwarn("collision_monitor: time moved backwards, resetting tf buffer")
				self.tf_buffer.clear()
				self.last_emergency = rospy.Time(0)
				rate = rospy.Rate(1.0 / self.heartbeat_period)
				continue
			except rospy.ROSInterruptException:
				break
			if (rospy.Time.now() - self.last_emergency).to_sec() > self.heartbeat_period:
				self.publish(MonitorStatus.OK, "Stop zone clear.")

	def cmd_callback(self, msg):
		self.speed = max(0.0, msg.linear.x)

	def cloud_callback(self, msg):
		try:
			tf = self.tf_buffer.lookup_transform(self.robot_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.05))
			cloud = do_transform_cloud(msg, tf)
		except tf2_ros.TransformException as e:
			rospy.logwarn_throttle(5.0, "collision_monitor: transform failed: %s" % e)
			return

		zone_length = self.margin + self.speed * self.horizon
		intruders = 0
		for x, y in point_cloud2.read_points(cloud, field_names=("x", "y"), skip_nans=True):
			if 0.0 < x < zone_length and abs(y) < self.halfwidth:
				intruders += 1

		if intruders >= self.min_points:
			self.publish(MonitorStatus.ESTOP, "Obstacle inside stop zone (%d points)." % intruders)
			self.last_emergency = rospy.Time.now()

	def publish(self, status, message):
		msg = MonitorStatus()
		msg.status = status
		msg.message = message
		self.status_pub.publish(msg)


if __name__ == "__main__":
	rospy.init_node("tinyhelm_collision_monitor")
	node = CollisionMonitor()
	node.spin()
