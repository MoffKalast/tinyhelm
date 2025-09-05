#!/usr/bin/env python3
import math
import rospy
import numpy as np
import threading
from collections import defaultdict

from std_msgs.msg import Empty
from geometry_msgs.msg import Point32 
from nav_msgs.msg import GridCells
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from rospy.exceptions import ROSTimeMovedBackwardsException

class SensorObstacleNode:
	def __init__(self):
		rospy.init_node("sensor_to_obstacle_node")

		self.FIXED_FRAME = rospy.get_param('~fixed_frame', 'local')
		self.GRID_SIZE = rospy.get_param('~obstacle_grid_size', 0.5)
		self.MIN_HITS = rospy.get_param('~min_hits_threshold', 3)

		self.pointcloud_sub = rospy.Subscriber('/obstacle_cloud', PointCloud2, self.pointcloud_callback)

		self.cells_pub = rospy.Publisher('/obstacle_grid', GridCells, queue_size=1)

		self.message = self.new_grid()

		# Hit counting for point cloud data
		self.scan_hit_counts = defaultdict(int)
		self.lock = threading.Lock()

	def new_grid(self):
		grid = GridCells()
		grid.header.frame_id = self.FIXED_FRAME
		grid.cell_width = self.GRID_SIZE
		grid.cell_height = self.GRID_SIZE
		grid.cells = []
		return grid

	def send(self):
		# Add cells that meet the minimum hit threshold
		with self.lock:
			for (x, y), count in self.scan_hit_counts.items():
				if count >= self.MIN_HITS:
					point = Point32()
					point.x = x * self.GRID_SIZE
					point.y = y * self.GRID_SIZE
					point.z = 0
					self.message.cells.append(point)

		if len(self.message.cells) == 0:
			return
		
		self.cells_pub.publish(self.message)
		self.message = self.new_grid()

	def pointcloud_callback(self, msg):
		# Check if the point cloud is in the fixed frame
		if msg.header.frame_id != self.FIXED_FRAME:
			rospy.logwarn_throttle(5.0, f"Rejecting PointCloud2 from frame '{msg.header.frame_id}' - expected '{self.FIXED_FRAME}'")
			return
		
		# Extract points from PointCloud2
		try:
			points = []
			for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
				points.append([point[0], point[1], point[2]])
			
			if len(points) == 0:
				return
			
			# Convert to numpy array for vectorized operations
			points_array = np.array(points)
			
			# Vectorized gridding - quantize to grid coordinates
			grid_coords = np.round(points_array[:, :2] / self.GRID_SIZE).astype(int)
			
			# Convert to set of tuples for deduplication
			unique_grid_cells = set(map(tuple, grid_coords))
			
			# Increment hit counts for each grid cell
			with self.lock:
				for (x, y) in unique_grid_cells:
					self.scan_hit_counts[(x, y)] += 1
					
		except Exception as e:
			rospy.logerr(f"Error processing PointCloud2: {e}")
			return

sensor_node = SensorObstacleNode()
rate = rospy.Rate(rospy.get_param('rate', 5.0))

while not rospy.is_shutdown():
	try:
		sensor_node.send()
		rate.sleep()
	except ROSTimeMovedBackwardsException as e:
		print(e)