#!/usr/bin/env python3
# Simulates a 2D lidar against user defined obstacle polygons, feeding the real
# scan_to_cloud -> obstacle_planner pipeline. Polygons come from a ~polygons param
# (list of [[x,y],...] rings in the fixed frame) and/or live PolygonStamped messages,
# so obstacles can be drawn from rviz or scripts during a run. A ghost_rate parameter
# injects spurious returns to emulate sun glint for geofence/budget regression tests.
import math
import rospy
import numpy as np
import tf2_ros
import copy

from std_msgs.msg import Empty, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion

class SimObstacles:

	def __init__(self):
		rospy.init_node("sim_obstacles")

		self.FIXED_FRAME = rospy.get_param("~fixed_frame", "local")
		self.LASER_FRAME = rospy.get_param("~laser_frame", "laser")

		self.N_BEAMS = rospy.get_param("~num_beams", 360)
		self.MAX_RANGE = rospy.get_param("~max_range", 50.0)
		self.MIN_RANGE = rospy.get_param("~min_range", 0.5)
		self.RATE = rospy.get_param("~rate", 10.0)

		self.NOISE_SIGMA = rospy.get_param("~noise_sigma", 0.05)
		self.DROPOUT = rospy.get_param("~dropout_probability", 0.02)
		self.SPECKLE_RATE = rospy.get_param("~speckle_rate", 1.0)

		self.segments = np.zeros((0, 4))
		self.rings = []
		for ring in rospy.get_param("~polygons", []):
			self.add_ring(np.array(ring, dtype=float))

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

		self.add_sub = rospy.Subscriber("/sim/add_obstacle_polygon", PolygonStamped, self.add_polygon)
		self.clear_sub = rospy.Subscriber("/sim/clear_obstacle_polygons", Empty, self.clear_polygons)

		self.scan_unreliable_pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=1)
		self.scan_verified_pub = rospy.Publisher("/scan_verified", LaserScan, queue_size=1)
		self.marker_pub = rospy.Publisher("/sim/obstacle_markers", MarkerArray, queue_size=1, latch=True)

		self.publish_markers()
		rospy.loginfo(f"sim_obstacles: {len(self.segments)} segments loaded, {self.N_BEAMS} beams @ {self.RATE}Hz")

	def add_ring(self, ring):
		if len(ring) < 3:
			return
		closed = np.vstack([ring, ring[0]])
		segs = np.hstack([closed[:-1], closed[1:]])
		self.segments = np.vstack([self.segments, segs])
		self.rings.append(ring)

	def publish_markers(self):
		arr = MarkerArray()
		wipe = Marker()
		wipe.header.frame_id = self.FIXED_FRAME
		wipe.action = Marker.DELETEALL
		arr.markers.append(wipe)

		for i, ring in enumerate(self.rings):
			m = Marker()
			m.header.frame_id = self.FIXED_FRAME
			m.header.stamp = rospy.Time.now()
			m.ns = "sim_obstacles"
			m.id = i
			m.type = Marker.LINE_STRIP
			m.action = Marker.ADD
			m.scale.x = 0.3
			m.color = ColorRGBA(0.9, 0.3, 0.1, 1.0)
			m.pose.orientation.w = 1.0
			for p in list(ring) + [ring[0]]:
				m.points.append(Point(p[0], p[1], 0.0))
			arr.markers.append(m)
		self.marker_pub.publish(arr)

	def add_polygon(self, msg):
		if msg.header.frame_id != self.FIXED_FRAME:
			rospy.logwarn(f"Ignoring polygon in frame '{msg.header.frame_id}', expected '{self.FIXED_FRAME}'")
			return
		self.add_ring(np.array([[p.x, p.y] for p in msg.polygon.points]))
		self.publish_markers()
		rospy.loginfo(f"sim_obstacles: polygon added, {len(self.segments)} segments total")

	def clear_polygons(self, msg):
		self.segments = np.zeros((0, 4))
		self.rings = []
		self.publish_markers()
		rospy.loginfo("sim_obstacles: polygons cleared")

	def get_sensor_pose(self):
		tf = self.tf2_buffer.lookup_transform(self.FIXED_FRAME, self.LASER_FRAME, rospy.Time(0), rospy.Duration(0.1))
		q = tf.transform.rotation
		_, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
		return tf.transform.translation.x, tf.transform.translation.y, yaw

	# Vectorized ray/segment intersection: for each beam direction the smallest positive hit
	# distance across all segments, inf where nothing is hit within max range.
	def raycast(self, ox, oy, angles):
		ranges = np.full(len(angles), np.inf)
		if len(self.segments) == 0:
			return ranges

		dx = np.cos(angles)[:, None]
		dy = np.sin(angles)[:, None]
		x1 = self.segments[None, :, 0] - ox
		y1 = self.segments[None, :, 1] - oy
		x2 = self.segments[None, :, 2] - ox
		y2 = self.segments[None, :, 3] - oy
		ex = x2 - x1
		ey = y2 - y1

		denom = dx * ey - dy * ex
		with np.errstate(divide='ignore', invalid='ignore'):
			t = (x1 * ey - y1 * ex) / denom
			u = (x1 * dy - y1 * dx) / denom

		valid = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
		t = np.where(valid, t, np.inf)
		return np.min(t, axis=1)

	def make_scan(self):
		try:
			ox, oy, yaw = self.get_sensor_pose()
			have_pose = True
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.logwarn_throttle(5.0, "sim_obstacles: no TF " + self.FIXED_FRAME + " -> " + self.LASER_FRAME + ", publishing speckle only")
			ox = oy = yaw = 0.0
			have_pose = False

		scan = LaserScan()
		scan.header.stamp = rospy.Time.now()
		scan.header.frame_id = self.LASER_FRAME
		scan.angle_min = -math.pi
		scan.angle_max = math.pi
		scan.angle_increment = 2.0 * math.pi / self.N_BEAMS
		scan.range_min = self.MIN_RANGE
		scan.range_max = self.MAX_RANGE

		beam_angles = scan.angle_min + np.arange(self.N_BEAMS) * scan.angle_increment

		if have_pose:
			true_ranges = self.raycast(ox, oy, beam_angles + yaw)
		else:
			true_ranges = np.full(self.N_BEAMS, np.inf)

		# ------------------------------------------------------------------
		# Unreliable scan (noise + dropout + speckle)
		# ------------------------------------------------------------------
		noisy_ranges = true_ranges.copy()

		if have_pose:
			noisy_ranges += np.random.normal(0.0, self.NOISE_SIGMA, self.N_BEAMS)
			noisy_ranges[np.random.random(self.N_BEAMS) < self.DROPOUT] = np.inf

		n_speckle = np.random.poisson(self.SPECKLE_RATE)
		if n_speckle > 0:
			idx = np.random.randint(0, self.N_BEAMS, n_speckle)
			noisy_ranges[idx] = np.random.uniform(self.MIN_RANGE * 2, self.MAX_RANGE * 0.8, n_speckle)

		noisy_ranges[(noisy_ranges < self.MIN_RANGE) | (noisy_ranges > self.MAX_RANGE)] = np.inf

		noisy_scan = copy.deepcopy(scan)
		noisy_scan.ranges = noisy_ranges.tolist()

		# ------------------------------------------------------------------
		# Verified scan (clean, forward 100° FOV only)
		# ------------------------------------------------------------------
		verified_ranges = true_ranges.copy()

		fov = math.radians(100.0)
		half_fov = fov * 0.5
		forward = math.pi / 2.0
		verified_ranges[np.abs(beam_angles - forward) > half_fov] = np.inf

		verified_ranges[(verified_ranges < self.MIN_RANGE) | (verified_ranges > self.MAX_RANGE)] = np.inf

		verified_scan = copy.deepcopy(scan)
		verified_scan.ranges = verified_ranges.tolist()

		return noisy_scan, verified_scan

	def run(self):
		rate = rospy.Rate(self.RATE)
		while not rospy.is_shutdown():
			noisy_scan, verified_scan = self.make_scan()
			self.scan_unreliable_pub.publish(noisy_scan)
			self.scan_verified_pub.publish(verified_scan)
			rate.sleep()

if __name__ == "__main__":
	try:
		SimObstacles().run()
	except rospy.ROSInterruptException:
		pass
