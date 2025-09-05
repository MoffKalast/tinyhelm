#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from collections import defaultdict

from geometry_msgs.msg import Point32
from nav_msgs.msg import GridCells
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs.point_cloud2 as pc2

class SensorObstacleNode:
    def __init__(self):
        rospy.init_node("sensor_to_obstacle_node")

        self.FIXED_FRAME = rospy.get_param('~fixed_frame', 'local')
        self.GRID_SIZE = rospy.get_param('~obstacle_grid_size', 0.5)
        self.CELL_RESOLUTION = rospy.get_param('~cell_resolution', 0.2)  # Should match C++ node
        self.MIN_HITS = rospy.get_param('~min_hits_threshold', 3)
        self.DECAY_RATE = rospy.get_param('~decay_rate', 0.98)

        # Ensure parameters are the correct type
        self.GRID_SIZE = float(self.GRID_SIZE)
        self.CELL_RESOLUTION = float(self.CELL_RESOLUTION)
        self.MIN_HITS = int(self.MIN_HITS)
        self.DECAY_RATE = float(self.DECAY_RATE)

        # Subscribe to pre-processed cells
        self.hit_cells_sub = rospy.Subscriber('/obstacle_cloud/add', PointCloud2, self.hit_cells_callback)
        self.clear_cells_sub = rospy.Subscriber('/obstacle_cloud/clear', PointCloud2, self.clear_cells_callback)
        
        self.cells_pub = rospy.Publisher('/obstacle_grid', GridCells, queue_size=1)
        self.debug_markers_pub = rospy.Publisher('/obstacle_debug_markers', MarkerArray, queue_size=1)

        # Persistent obstacle grid
        self.obstacle_grid = GridCells()
        self.obstacle_grid.header.frame_id = self.FIXED_FRAME
        self.obstacle_grid.cell_width = self.GRID_SIZE
        self.obstacle_grid.cell_height = self.GRID_SIZE
        self.obstacle_grid.cells = []

        # 3D grid: (x, y, z) -> {'hits': count, 'clears': count}
        self.internal_grid = defaultdict(lambda: {'hits': 0.0, 'clears': 0.0})
        self.obstacle_cells = set()
        self.lock = threading.Lock()

        rospy.loginfo(f"Obstacle node initialized:")
        rospy.loginfo(f"  Fixed frame: {self.FIXED_FRAME}")
        rospy.loginfo(f"  Obstacle grid size: {self.GRID_SIZE}m")
        rospy.loginfo(f"  Cell resolution: {self.CELL_RESOLUTION}m")
        rospy.loginfo(f"  Min hits threshold: {self.MIN_HITS}")

    def hit_cells_callback(self, msg):
        if msg.header.frame_id != self.FIXED_FRAME:
            rospy.logwarn_throttle(5.0, f"Rejecting hit cells from frame '{msg.header.frame_id}'")
            return

        try:
            with self.lock:
                # Process hit cells (already quantized by C++ node)
                for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    # Further quantize to our cell resolution if needed
                    x = round(point[0] / self.CELL_RESOLUTION)
                    y = round(point[1] / self.CELL_RESOLUTION)
                    z = round(point[2] / self.CELL_RESOLUTION)
                    
                    self.internal_grid[(x, y, z)]['hits'] += 1.0

        except Exception as e:
            rospy.logerr(f"Error processing hit cells: {e}")

    def clear_cells_callback(self, msg):
        if msg.header.frame_id != self.FIXED_FRAME:
            rospy.logwarn_throttle(5.0, f"Rejecting clear cells from frame '{msg.header.frame_id}'")
            return

        try:
            with self.lock:
                # Process clear cells
                for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                    # Quantize to our cell resolution
                    x = round(point[0] / self.CELL_RESOLUTION)
                    y = round(point[1] / self.CELL_RESOLUTION)
                    z = round(point[2] / self.CELL_RESOLUTION)
                    
                    # Only increment clears if we don't have strong hit evidence
                    cell_data = self.internal_grid[(x, y, z)]
                    if cell_data['hits'] <= cell_data['clears'] + 2:
                        cell_data['clears'] += 0.5  # Weight clears less than hits

        except Exception as e:
            rospy.logerr(f"Error processing clear cells: {e}")

    def publish_debug_markers(self):
        """Publish debug visualization of the internal 3D grid"""
        marker_array = MarkerArray()
        
        with self.lock:
            marker_id = 0
            for (ix, iy, iz), data in self.internal_grid.items():
                if data['hits'] > 0.1 or data['clears'] > 0.1:  # Only show cells with activity
                    marker = Marker()
                    marker.header.frame_id = self.FIXED_FRAME
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "obstacle_debug"
                    marker.id = marker_id
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # Position
                    marker.pose.position.x = ix * self.CELL_RESOLUTION
                    marker.pose.position.y = iy * self.CELL_RESOLUTION
                    marker.pose.position.z = iz * self.CELL_RESOLUTION
                    marker.pose.orientation.w = 1.0
                    
                    # Size
                    marker.scale.x = self.CELL_RESOLUTION * 0.9  # Slightly smaller for visibility
                    marker.scale.y = self.CELL_RESOLUTION * 0.9
                    marker.scale.z = self.CELL_RESOLUTION * 0.9
                    
                    # Color based on hit/clear ratio
                    hits = data['hits']
                    clears = data['clears']
                    total = hits + clears
                    
                    if total > 0:
                        hit_ratio = hits / total
                        marker.color.r = hit_ratio  # More red = more hits
                        marker.color.g = 1.0 - hit_ratio  # More green = more clears
                        marker.color.b = 0.0
                        marker.color.a = min(0.8, total * 0.1)  # Transparency based on total observations
                    else:
                        marker.color.r = 0.5
                        marker.color.g = 0.5
                        marker.color.b = 0.5
                        marker.color.a = 0.3
                    
                    marker.lifetime = rospy.Duration(2.0)
                    marker_array.markers.append(marker)
                    marker_id += 1
        
        # Clear old markers if no new ones
        if len(marker_array.markers) == 0:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.FIXED_FRAME
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "obstacle_debug"
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
        
        self.debug_markers_pub.publish(marker_array)

    def update_obstacle_grid(self):
        """Update the persistent obstacle grid"""
        with self.lock:
            new_obstacle_cells = set()
            
            # Group 3D cells by 2D grid coordinates
            grid_2d_groups = defaultdict(list)
            for (ix, iy, iz), data in self.internal_grid.items():
                ox = int(ix * self.CELL_RESOLUTION / self.GRID_SIZE)
                oy = int(iy * self.CELL_RESOLUTION / self.GRID_SIZE)
                grid_2d_groups[(ox, oy)].append(data)
            
            # Determine obstacle cells
            for (ox, oy), cell_data_list in grid_2d_groups.items():
                total_hits = sum(data['hits'] for data in cell_data_list)
                total_clears = sum(data['clears'] for data in cell_data_list)
                
                if total_hits >= self.MIN_HITS and total_hits > total_clears:
                    new_obstacle_cells.add((ox, oy))
            
            # Update grid if changed
            if new_obstacle_cells != self.obstacle_cells:
                self.obstacle_cells = new_obstacle_cells
                self.obstacle_grid.cells = []
                
                for (ox, oy) in self.obstacle_cells:
                    point = Point32()
                    point.x = ox * self.GRID_SIZE
                    point.y = oy * self.GRID_SIZE
                    point.z = 0
                    self.obstacle_grid.cells.append(point)

    def apply_decay(self):
        """Apply decay to old observations"""
        with self.lock:
            cells_to_remove = []
            for cell_key, data in self.internal_grid.items():
                data['hits'] *= self.DECAY_RATE
                data['clears'] *= self.DECAY_RATE
                
                if data['hits'] < 0.1 and data['clears'] < 0.1:
                    cells_to_remove.append(cell_key)
            
            for cell_key in cells_to_remove:
                del self.internal_grid[cell_key]

    def send(self):
        self.update_obstacle_grid()
        
        if len(self.obstacle_grid.cells) > 0:
            self.obstacle_grid.header.stamp = rospy.Time.now()
            self.cells_pub.publish(self.obstacle_grid)
        
        # Publish debug markers
        self.publish_debug_markers()
        
        # Apply decay periodically
        if rospy.get_time() % 5 < 1.0:
            self.apply_decay()

if __name__ == "__main__":
    sensor_node = SensorObstacleNode()
    rate = rospy.Rate(rospy.get_param('~rate', 1.0))

    while not rospy.is_shutdown():
        try:
            sensor_node.send()
            rate.sleep()
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")