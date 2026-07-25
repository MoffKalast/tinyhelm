#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray
from tinyhelm_core.msg import MonitorStatus

import corridor_planner
from chunked_grid import ChunkedGrid
from cost_field import CostField
from corridor_planner import CorridorPlanner

STATUS_LEVELS = {
	corridor_planner.OK: MonitorStatus.OK,
	corridor_planner.WARN: MonitorStatus.WARN,
	corridor_planner.REPLAN: MonitorStatus.REPLAN,
	corridor_planner.ERROR: MonitorStatus.OBSERVED_ERROR,
}


class ObstaclePlannerNode:
	"""Observes the strategic mission (/tinyhelm/mission) and the line planner's remaining
	tactical path, maintains a decaying obstacle grid around the vessel, and when the
	tactical corridor is intruded plans a detour with Theta* through the remaining
	strategic waypoints. Space beyond the loaded grid window is treated as clear. This
	node owns everything ROS: topics, TF, message conversion and the tick loop; all
	planning logic lives in CorridorPlanner. It never commands anything: it publishes a
	proposed path and a MonitorStatus, and the helm core decides what to do with them."""

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local")
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")

		self.tactical_plan_topic = rospy.get_param("~tactical_plan_topic", "/waypoints/_plan")
		self.divergence_param = rospy.get_param("~divergence_param", "/tinyhelm_waypoints/max_line_divergence")

		self.res = rospy.get_param("~resolution", 0.5)
		self.chunk_size = rospy.get_param("~chunk_size", 32.0)
		self.load_radius = rospy.get_param("~load_radius", 100.0)

		self.hit_delta = rospy.get_param("~hit_delta", 16)
		self.occ_thresh = rospy.get_param("~occupied_threshold", 70)
		self.half_life = rospy.get_param("~decay_half_life", 300.0)

		self.inflate_radius = rospy.get_param("~inflate_radius", 1.5)
		self.soft_radius = rospy.get_param("~soft_radius", 5.0)
		self.soft_weight = rospy.get_param("~soft_weight", 2.0)

		self.monitor_rate = rospy.get_param("~monitor_rate", 10.0)
		self.pose_jump_threshold = rospy.get_param("~pose_jump_threshold", 10.0)
		self.grid_publish_period = rospy.get_param("~grid_publish_period", 2.0)

		config = {
			"resolution": self.res,
			"inflate_radius": self.inflate_radius,
			"min_detour": rospy.get_param("~min_detour", 25.0),
			"detour_leg_fraction": rospy.get_param("~detour_leg_fraction", 0.5),
			"max_detour": rospy.get_param("~max_detour", 200.0),
			"budget_factor": rospy.get_param("~budget_factor", 3.0),
			"unreachable_cycles": rospy.get_param("~unreachable_cycles", 3),
			"waypoint_reached_radius": rospy.get_param("~waypoint_reached_radius", 6.0),
		}

		self.grid = ChunkedGrid(self.res, self.chunk_size, self.hit_delta, self.occ_thresh, self.half_life)
		self.planner = CorridorPlanner(config, self.publish_status,
			lambda m: rospy.loginfo("obstacle_planner: %s" % m),
			lambda m: rospy.logwarn("obstacle_planner: %s" % m))

		self.local_field = CostField()
		self.blocked = False
		self.have_last_pose = False
		self.last_rx = 0.0
		self.last_ry = 0.0
		self.last_grid_publish = rospy.Time(0)

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		rospy.Subscriber("/reliable_cloud", PointCloud2, self.reliable_hits_callback, queue_size=5)
		rospy.Subscriber("/unreliable_cloud", PointCloud2, self.unreliable_hits_callback, queue_size=5)
		rospy.Subscriber("/free_cloud", PointCloud2, self.free_hits_callback, queue_size=5)

		rospy.Subscriber("/obstacle_grid/clear", Empty, self.grid_clear_callback, queue_size=1)
		rospy.Subscriber("/tinyhelm/mission", Path, self.mission_callback, queue_size=1)
		rospy.Subscriber(self.tactical_plan_topic, Path, self.tactical_callback, queue_size=1)

		self.path_pub = rospy.Publisher("/obstacle_planner/path", Path, queue_size=1, latch=True)
		self.remaining_pub = rospy.Publisher("/obstacle_planner/remaining", Path, queue_size=1, latch=True)
		self.status_pub = rospy.Publisher("/tinyhelm/monitor/obstacles", MonitorStatus, queue_size=5, latch=True)
		self.local_grid_pub = rospy.Publisher("/obstacle_planner/grid_local", OccupancyGrid, queue_size=1, latch=True)

		# Placeholder so the helm's marker aggregation has something to subscribe to; nothing is
		# published on it yet, the geofence and proposed detour are the obvious first candidates
		self.markers_pub = rospy.Publisher("/obstacles/_markers", MarkerArray, queue_size=1)

		rospy.loginfo("obstacle_planner: res %.2fm, chunks %.0fm, load radius %.0fm, frame %s", self.res, self.chunk_size, self.load_radius, self.planning_frame)

	def free_hits_callback(self, msg):
		if msg.header.frame_id != self.planning_frame:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
				msg = do_transform_cloud(msg, tf)
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "obstacle_planner: cloud transform failed: %s" % e)
				return
			
		now = rospy.Time.now().to_sec()
		for x, y in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
			self.grid.remove_hit(x, y, now)

	def reliable_hits_callback(self, msg):
		if msg.header.frame_id != self.planning_frame:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
				msg = do_transform_cloud(msg, tf)
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "obstacle_planner: cloud transform failed: %s" % e)
				return

		now = rospy.Time.now().to_sec()
		for x, y in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
			self.grid.add_hit(x, y, now)

	def unreliable_hits_callback(self, msg):
		if msg.header.frame_id != self.planning_frame:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
				msg = do_transform_cloud(msg, tf)
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "obstacle_planner: cloud transform failed: %s" % e)
				return

		now = rospy.Time.now().to_sec()
		for x, y in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
			self.grid.add_unreliable_hit(x, y, now)

	def hits_callback(self, msg):
		if msg.header.frame_id != self.planning_frame:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
				msg = do_transform_cloud(msg, tf)
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "obstacle_planner: cloud transform failed: %s" % e)
				return

		now = rospy.Time.now().to_sec()
		for x, y in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
			self.grid.add_hit(x, y, now)

	def grid_clear_callback(self, _):
		self.grid.clear()
		rospy.loginfo("obstacle_planner: grid cleared by request")

	def mission_callback(self, msg):
		# Missions can arrive in any frame; everything downstream assumes the planning
		# frame, and untransformed coordinates silently monitor and plan in the wrong place
		poses = list(msg.poses)
		frame = msg.header.frame_id
		if frame and frame != self.planning_frame and poses:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, frame, rospy.Time(0))
				poses = [tf2_geometry_msgs.do_transform_pose(p, tf) for p in poses]
			except tf2_ros.TransformException as e:
				rospy.logwarn("obstacle_planner: mission in frame '%s' could not be transformed, ignoring it: %s" % (frame, e))
				poses = []

		self.blocked = False
		self.planner.set_mission([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses])

	def tactical_callback(self, msg):
		self.planner.set_tactical([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses])

	def get_robot_pose(self):
		try:
			tf = self.tf_buffer.lookup_transform(self.planning_frame, self.robot_frame, rospy.Time(0))
			return tf.transform.translation.x, tf.transform.translation.y
		except tf2_ros.TransformException:
			return None

	def tick(self):
		pose = self.get_robot_pose()
		if pose is None:
			return
		rx, ry = pose
		now = rospy.Time.now()

		if self.have_last_pose and math.hypot(rx - self.last_rx, ry - self.last_ry) > self.pose_jump_threshold:
			rospy.logwarn("obstacle_planner: pose jumped %.1fm, assuming ENU reset and clearing grid", math.hypot(rx - self.last_rx, ry - self.last_ry))
			self.grid.clear()
		self.last_rx = rx
		self.last_ry = ry
		self.have_last_pose = True

		self.grid.maintain(rx, ry, self.load_radius, now.to_sec())

		if (now - self.last_grid_publish).to_sec() >= self.grid_publish_period:
			self.publish_local_grid(rx, ry)
			self.last_grid_publish = now

		if not self.planner.mission:
			return
		self.planner.update_remaining(rx, ry)
		if not self.planner.remaining:
			return
		self.publish_remaining(now)

		corridor = rospy.get_param(self.divergence_param, 0.0)
		self.planner.effective_inflate = self.inflate_radius + corridor

		fence = self.planner.build_geofence(rx, ry)
		self.build_local_field(rx, ry, fence)

		# Nothing to monitor until the controller reports a tactical plan; the helm feeds
		# the mission to the controller directly, so there is no bootstrap replan
		if len(self.planner.tactical) < 2:
			return

		if self.planner.corridor_clear(self.local_field):
			if self.blocked:
				rospy.loginfo("obstacle_planner: corridor clear again")
			self.blocked = False
			self.publish_status(corridor_planner.OK, "Corridor clear.")
			return

		self.blocked = True

		# Only produce a new correction when genuinely needed: as long as the vessel is
		# still on the active correction and its remainder is obstacle-free, there is
		# nothing to fix
		if self.planner.published_still_valid(self.local_field, rx, ry):
			self.publish_status(corridor_planner.REPLAN, "Active correction still clear.")
			return

		points = self.planner.replan(self.local_field, rx, ry)
		if points:
			self.publish_path(points, now)

	def build_local_field(self, rx, ry, fence):
		size = int(math.ceil(2.0 * self.load_radius / self.res))
		ox = math.floor((rx - self.load_radius) / self.res) * self.res
		oy = math.floor((ry - self.load_radius) / self.res) * self.res
		occupied = self.grid.window(ox, oy, size) >= self.occ_thresh
		self.local_field.build(self.res, ox, oy, occupied, self.planner.effective_inflate, self.soft_radius, self.soft_weight, fence)

	def make_pose(self, x, y, z):
		p = PoseStamped()
		p.header.frame_id = self.planning_frame
		p.pose.position.x = x
		p.pose.position.y = y
		p.pose.position.z = z
		p.pose.orientation.w = 1.0
		return p

	def publish_path(self, points, stamp):
		msg = Path()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = stamp
		msg.poses = [self.make_pose(x, y, z) for x, y, z in points]
		self.path_pub.publish(msg)

	def publish_remaining(self, stamp):
		msg = Path()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = stamp
		msg.poses = [self.make_pose(x, y, z) for x, y, z in self.planner.remaining]
		self.remaining_pub.publish(msg)

	def publish_status(self, level, message):
		msg = MonitorStatus()
		msg.status = STATUS_LEVELS[level]
		msg.message = message
		self.status_pub.publish(msg)

	def publish_local_grid(self, rx, ry):
		if self.local_grid_pub.get_num_connections() == 0:
			return
		size = int(math.ceil(2.0 * self.load_radius / self.res))
		ox = math.floor((rx - self.load_radius) / self.res) * self.res
		oy = math.floor((ry - self.load_radius) / self.res) * self.res

		values = self.grid.window(ox, oy, size).astype(np.int16)
		scaled = np.clip(values * 98 // self.occ_thresh, 1, 98)
		data = np.where(values >= self.occ_thresh, 100, np.where(values > 0, scaled, -1))

		msg = OccupancyGrid()
		msg.header.frame_id = self.planning_frame
		msg.header.stamp = rospy.Time.now()
		msg.info.resolution = self.res
		msg.info.width = size
		msg.info.height = size
		msg.info.origin.position.x = ox
		msg.info.origin.position.y = oy
		msg.info.origin.orientation.w = 1.0
		msg.data = data.astype(np.int8).flatten().tolist()
		self.local_grid_pub.publish(msg)

	def spin(self):
		"""Own tick loop instead of rospy.Timer so a sim time jump backwards (rosbag
		reset) is caught here instead of stalling a timer thread until its stale target
		time comes around again."""
		rate = rospy.Rate(self.monitor_rate)
		while not rospy.is_shutdown():
			try:
				rate.sleep()
			except rospy.ROSTimeMovedBackwardsException:
				self.handle_time_jump()
				rate = rospy.Rate(self.monitor_rate)
				continue
			except rospy.ROSInterruptException:
				break
			self.tick()

	def handle_time_jump(self):
		rospy.logwarn("obstacle_planner: time moved backwards, resetting tf buffer and grid")
		self.tf_buffer.clear()
		self.grid.clear()
		self.have_last_pose = False
		self.last_grid_publish = rospy.Time(0)
		self.planner.last_published = []
		self.planner.unreachable_counts.clear()


if __name__ == "__main__":
	rospy.init_node("tinyhelm_obstacles")
	node = ObstaclePlannerNode()
	node.spin()
