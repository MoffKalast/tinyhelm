#!/usr/bin/env python3
import threading

import rospy
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty

from cloud import cloud_xy
from cost_field import encode_cost
from local_grid import LocalGrid

class CostmapNode:
	"""Owns everything to do with obstacle evidence: the clouds, the rolling window, decay, and the
	distance field. Publishes that field as a plain nav_msgs/OccupancyGrid carrying cost on
	the usual convention, so it needs no message of its own and renders in rviz as an ordinary
	costmap.

	Ingestion happens on the subscriber threads because it is bounded and per message. Maintenance
	and publishing happen on one timer, so decay keeps running and the map keeps fading even if the
	sensors go quiet, which is exactly when a stale map is most dangerous."""

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local") 
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")

		self.params = rospy.get_param("/tinyhelm_obstacles", {})
		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_obstacles'. Did you load the YAML file?")
			raise SystemExit(1)

		self.res = self.params.get('costmap_resolution')
		self.extent = self.params.get('costmap_size')
		self.soft_radius = self.params.get('soft_radius')

		size_cells = int(round(self.extent / self.res))

		self.lock = threading.Lock()
		self.grid = LocalGrid(
			self.res,
			size_cells,
			soft_radius=self.soft_radius,
			confirm_seconds=self.params.get('confirm_seconds'),
			memory_seconds=self.params.get('memory_seconds'),
			grace_seconds=self.params.get('grace_seconds'),
			forget_ratio=self.params.get('forget_ratio'),
			confirm_period=self.params.get('confirm_period'),
			scroll_hysteresis_cells=self.params.get('scroll_hysteresis_cells'),
		)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.grid_pub = rospy.Publisher("/obstacles/costmap", OccupancyGrid, queue_size=1, latch=True)

		rospy.Subscriber("/reliable_cloud", PointCloud2, self.reliable_callback, queue_size=5)
		rospy.Subscriber("/unreliable_cloud", PointCloud2, self.unreliable_callback, queue_size=5)
		rospy.Subscriber("/free_cloud", PointCloud2, self.free_callback, queue_size=5)
		rospy.Subscriber("/obstacles/clear_costmap", Empty, self.clear_callback, queue_size=1)

		rospy.loginfo("costmap: %.0fm window at %.2fm (%d cells), soft radius %.1fm, frame %s", self.extent, self.res, size_cells, self.soft_radius, self.planning_frame)

		self.timer = rospy.Timer(rospy.Duration(0.1), self.maintain)

	def robot_position(self):
		"""A pose is not optional: without one there is no window to maintain. Waits rather than
		giving up, since a gap here is a localisation dropout and the vessel is not going anywhere
		useful until it clears."""
		while not rospy.is_shutdown():
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, self.robot_frame, rospy.Time(0))
				return tf.transform.translation.x, tf.transform.translation.y
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "costmap: waiting for %s -> %s: %s" % (self.planning_frame, self.robot_frame, e))
				rospy.sleep(0.2)

		return None

	def cloud_to_xy(self, msg):
		if msg.header.frame_id == self.planning_frame:
			return cloud_xy(msg)

		try:
			tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
		except tf2_ros.TransformException as e:
			rospy.logwarn_throttle(5.0, "costmap: cloud transform failed: %s" % e)
			return None

		return cloud_xy(msg, tf.transform)

	def ingest(self, msg, observe):
		xy = self.cloud_to_xy(msg)
		if xy is None:
			return

		now = rospy.Time.now().to_sec()
		with self.lock:
			if not self.grid.placed:
				return
			self.grid.mark(observe(xy[0], xy[1], now))

	def reliable_callback(self, msg):
		self.ingest(msg, self.grid.observe_reliable)

	def unreliable_callback(self, msg):
		self.ingest(msg, self.grid.observe_unreliable)

	def free_callback(self, msg):
		self.ingest(msg, self.grid.observe_free)

	def clear_callback(self, _):
		with self.lock:
			self.grid.clear()
		rospy.loginfo("costmap: cleared by request")

	def maintain(self, _):
		position = self.robot_position()
		if position is None:
			return

		now = rospy.Time.now().to_sec()
		with self.lock:
			scrolled = self.grid.recentre(position[0], position[1])
			self.grid.mark(self.grid.settle(now))
			if scrolled:
				self.grid.refresh_distance(force=True)
			else:
				self.grid.refresh_distance()

			snapshot = self.grid.dist.copy()
			origin_x, origin_y = self.grid.origin_x, self.grid.origin_y

		self.publish(snapshot, origin_x, origin_y)

	def publish(self, dist, origin_x, origin_y):
		msg = OccupancyGrid()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = rospy.Time.now()
		msg.info.resolution = self.res
		msg.info.width = dist.shape[1]
		msg.info.height = dist.shape[0]
		msg.info.origin.position.x = origin_x
		msg.info.origin.position.y = origin_y
		msg.info.origin.orientation.w = 1.0

		# Encoded with no hard inflation, so what goes out is the distance field itself and every
		# consumer applies its own clearance. Baking a radius in here would pin the planner to one
		# vessel geometry and lose the distances inside it.
		msg.data = encode_cost(dist, 0.0, self.soft_radius).ravel().tolist()

		self.grid_pub.publish(msg)

if __name__ == "__main__":
	rospy.init_node("tinyhelm_costmap")
	CostmapNode()
	rospy.spin()
