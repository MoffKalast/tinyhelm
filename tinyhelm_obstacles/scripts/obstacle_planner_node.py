#!/usr/bin/env python3
import math
import rospy
import threading
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray
from tinyhelm_core.msg import MonitorStatus

import corridor_planner
from chunked_grid import ChunkedGrid
from cost_field import CostField
from corridor_planner import CorridorPlanner

RELIABLE, UNRELIABLE, FREE = "reliable", "unreliable", "free"

STATUS_LEVELS = {
	corridor_planner.OK: MonitorStatus.OK,
	corridor_planner.WARN: MonitorStatus.WARN,
	corridor_planner.REPLAN: MonitorStatus.REPLAN,
	corridor_planner.ERROR: MonitorStatus.OBSERVED_ERROR,
}


class ObstaclePlannerNode:
	"""Observes the mission being executed and the path the vessel is currently following, both
	fed in by the helm, maintains a decaying obstacle grid around the vessel, and when the
	current corridor is intruded plans a detour with Theta* through the remaining mission
	waypoints. Space beyond the loaded grid window is treated as clear. This node owns
	everything ROS: topics, TF, message conversion and the tick loop; all planning logic lives
	in CorridorPlanner. It never commands anything: it publishes a revised path and a
	MonitorStatus, and the helm core decides what to do with them."""

	def __init__(self):
		self.planning_frame = rospy.get_param("/planning_frame", "local")
		self.robot_frame = rospy.get_param("/robot_frame", "base_link")

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
			"max_lateral_detour": rospy.get_param("~max_lateral_detour", 20.0),
			"budget_factor": rospy.get_param("~budget_factor", 3.0),
			"unreachable_cycles": rospy.get_param("~unreachable_cycles", 3),
			"expansion_limit": rospy.get_param("~expansion_limit", 5000),
			"waypoint_reached_radius": rospy.get_param("~waypoint_reached_radius", 6.0),
		}

		self.grid = ChunkedGrid(self.res, self.chunk_size, self.hit_delta, self.occ_thresh, self.half_life)
		self.planner = CorridorPlanner(config, self.publish_status,
			lambda m: rospy.loginfo("obstacle_planner: %s" % m),
			lambda m: rospy.logwarn("obstacle_planner: %s" % m))

		self.local_field = CostField()

		# Subscriber callbacks only stash their input; everything that mutates the grid or the
		# planner is applied on the tick thread in apply_pending. replan() walks the mission for as
		# long as a search takes, and a mission cleared from a callback thread partway through that
		# walk used to take the node down with an IndexError.
		self.pending_lock = threading.Lock()
		self.pending_mission = None
		self.pending_current_path = None
		self.pending_grid_clear = False
		self.pending_hits = []
		self.max_pending_hit_batches = rospy.get_param("~max_pending_hit_batches", 200)

		self.fence = None
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

		rospy.Subscriber("/obstacles/_clear_grid", Empty, self.grid_clear_callback, queue_size=1)
		rospy.Subscriber("/obstacles/_mission_in", Path, self.mission_callback, queue_size=1)
		rospy.Subscriber("/obstacles/_current_path_in", Path, self.current_path_callback, queue_size=1)

		self.path_pub = rospy.Publisher("/obstacles/_revised_path_out", Path, queue_size=1, latch=True)
		self.status_pub = rospy.Publisher("/obstacles/_status", MonitorStatus, queue_size=5, latch=True)
		self.remaining_pub = rospy.Publisher("/obstacles/_remaining", Path, queue_size=1, latch=True)
		self.local_grid_pub = rospy.Publisher("/obstacles/_grid", OccupancyGrid, queue_size=1, latch=True)

		self.markers_pub = rospy.Publisher("/obstacles/_markers", MarkerArray, queue_size=1)

		rospy.loginfo("obstacle_planner: res %.2fm, chunks %.0fm, load radius %.0fm, frame %s", self.res, self.chunk_size, self.load_radius, self.planning_frame)

	def cloud_to_points(self, msg):
		"""Transform and unpack on the callback thread, since that part is bounded and per message,
		but hand the points over rather than touching the grid the tick thread is reading."""
		if msg.header.frame_id != self.planning_frame:
			try:
				tf = self.tf_buffer.lookup_transform(self.planning_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
				msg = do_transform_cloud(msg, tf)
			except tf2_ros.TransformException as e:
				rospy.logwarn_throttle(5.0, "obstacle_planner: cloud transform failed: %s" % e)
				return None

		return list(point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True))

	def queue_hits(self, kind, msg):
		points = self.cloud_to_points(msg)
		if points is None:
			return

		# Timestamped now rather than at apply time, so a slow tick does not make old returns look
		# fresher than they are to the decay
		batch = (kind, points, rospy.Time.now().to_sec())

		with self.pending_lock:
			self.pending_hits.append(batch)
			if len(self.pending_hits) > self.max_pending_hit_batches:
				dropped = len(self.pending_hits) - self.max_pending_hit_batches
				self.pending_hits = self.pending_hits[dropped:]
				rospy.logwarn_throttle(5.0, "obstacle_planner: tick is falling behind, dropped %d cloud batches" % dropped)

	def free_hits_callback(self, msg):
		self.queue_hits(FREE, msg)

	def reliable_hits_callback(self, msg):
		self.queue_hits(RELIABLE, msg)

	def unreliable_hits_callback(self, msg):
		self.queue_hits(UNRELIABLE, msg)

	def grid_clear_callback(self, _):
		with self.pending_lock:
			self.pending_grid_clear = True

	def apply_pending(self):
		"""Drains everything the callbacks stashed. Runs on the tick thread only, which is what
		makes the grid and the planner single threaded despite the multithreaded subscribers."""
		with self.pending_lock:
			mission = self.pending_mission
			current_path = self.pending_current_path
			hits = self.pending_hits
			clear = self.pending_grid_clear

			self.pending_mission = None
			self.pending_current_path = None
			self.pending_hits = []
			self.pending_grid_clear = False

		if clear:
			self.grid.clear()
			rospy.loginfo("obstacle_planner: grid cleared by request")

		for kind, points, stamp in hits:
			if kind == RELIABLE:
				for x, y in points:
					self.grid.add_hit(x, y, stamp)
			elif kind == UNRELIABLE:
				for x, y in points:
					self.grid.add_unreliable_hit(x, y, stamp)
			else:
				for x, y in points:
					self.grid.remove_hit(x, y, stamp)

		if mission is not None:
			self.blocked = False
			self.planner.set_mission(mission)

		if current_path is not None:
			self.planner.set_current_path(current_path)

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

		with self.pending_lock:
			self.pending_mission = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]

		#TODO apply mission before setting this otherwise it's garbage data
		#set up geofence radius
		rx, ry = self.get_robot_pose()
		self.fence = self.planner.build_geofence(rx, ry)
		self.publish_fence_markers()

		#update corridor width in case it was reconfigured
		corridor = rospy.get_param(self.divergence_param, 0.0)
		self.planner.effective_inflate = self.inflate_radius + corridor


	def current_path_callback(self, msg):
		with self.pending_lock:
			self.pending_current_path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]

	def get_robot_pose(self):
		try:
			tf = self.tf_buffer.lookup_transform(self.planning_frame, self.robot_frame, rospy.Time(0))
			return tf.transform.translation.x, tf.transform.translation.y
		except tf2_ros.TransformException:
			rospy.sleep(1.0)
			return self.get_robot_pose()

	def tick(self):
		self.apply_pending()
		
		rx, ry = self.get_robot_pose()
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
		self.build_local_field(rx, ry)

		# Nothing to monitor until the helm relays a current path; the mission is handed to the
		# controller directly, so there is no bootstrap replan
		if len(self.planner.current_path) < 2:
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


	def publish_fence_markers(self):

		def fence_marker(capsule, marker_id, stamp):
			"""Outline of one geofence capsule as a stadium: the search cannot leave this, so its radius
			is what actually decides how much space a replan has to chew through."""
			marker = Marker()
			marker.header.frame_id = self.planning_frame
			marker.header.stamp = stamp
			marker.ns = "geofence"
			marker.id = marker_id
			marker.type = Marker.LINE_STRIP
			marker.action = Marker.ADD
			marker.scale.x = 0.3
			marker.pose.orientation.w = 1.0
			marker.lifetime = rospy.Duration(0)
			marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.6, 1.0, 0.7

			dx, dy = capsule.x2 - capsule.x1, capsule.y2 - capsule.y1
			length = math.hypot(dx, dy)
			ux, uy = (dx / length, dy / length) if length > 1e-6 else (1.0, 0.0)
			nx, ny = -uy, ux
			r = capsule.radius
			arc = 20

			# Half turn around the far end, then half turn around the near end, closing the loop. The
			# straight flanks fall out of joining the two arcs.
			points = []
			for end_x, end_y, base in ((capsule.x2, capsule.y2, 0.0), (capsule.x1, capsule.y1, math.pi)):
				for i in range(arc + 1):
					theta = base + math.pi * i / arc
					c, sn = math.cos(theta), math.sin(theta)
					points.append(Point(end_x + r * (c * nx + sn * ux), end_y + r * (c * ny + sn * uy), 0.0))

			points.append(points[0])
			marker.points = points
			return marker

		now = rospy.Time.now()
		arr = MarkerArray()
		for i, capsule in enumerate(self.fence):
			arr.markers.append(fence_marker(capsule, i, now))

		self.markers_pub.publish(arr)

	def build_local_field(self, rx, ry):
		size = int(math.ceil(2.0 * self.load_radius / self.res))
		ox = math.floor((rx - self.load_radius) / self.res) * self.res
		oy = math.floor((ry - self.load_radius) / self.res) * self.res
		occupied = self.grid.window(ox, oy, size) >= self.occ_thresh
		self.local_field.build(self.res, ox, oy, occupied, self.planner.effective_inflate, self.soft_radius, self.soft_weight, self.fence)

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
