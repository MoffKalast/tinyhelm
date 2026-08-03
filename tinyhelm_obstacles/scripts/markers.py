#!/usr/bin/env python3
import rospy

from shapely.geometry import LineString
from shapely.ops import unary_union

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from tinyhelm_core.msg import MonitorStatus

STATUS_COLOURS = {
	MonitorStatus.OK: (0.2, 0.6, 1.0), #blue
	MonitorStatus.REPLAN: (0.16, 0.74, 0.44), #green
	MonitorStatus.SLOW: (1.0, 0.8, 0.1), #yellow
	MonitorStatus.HOLD: (1.0, 0.25, 0.2), #red
	MonitorStatus.ESTOP: (0.55, 0.0, 0.94) #purple
}

class DebugMarkers:

	def __init__(self, planning_frame, max_detour):
		self.planning_frame = planning_frame
		self.max_detour = max_detour
		self.markers_pub = rospy.Publisher("/obstacles/_markers", MarkerArray, queue_size=1, latch=True)
		self.segments = []

	def corridor_silhouette(self, polyline):
		union = unary_union([LineString(polyline).buffer(self.max_detour, resolution=8)])

		segments = []
		for geom in getattr(union, "geoms", (union,)):
			for ring in [geom.exterior] + list(geom.interiors):
				coords = list(ring.coords)
				segments.extend((coords[i - 1], coords[i]) for i in range(1, len(coords)))

		return segments

	def corridor_marker(self, segments, stamp, status):
		marker = Marker()
		marker.header.frame_id = self.planning_frame
		marker.header.stamp = stamp
		marker.ns = "corridor"
		marker.id = 0
		marker.type = Marker.LINE_LIST
		marker.action = Marker.ADD
		marker.scale.x = 0.2
		marker.pose.orientation.w = 1.0
		marker.color.r, marker.color.g, marker.color.b = STATUS_COLOURS.get(status, STATUS_COLOURS[MonitorStatus.OK])
		marker.color.a = 0.7

		for a, b in segments:
			marker.points.append(Point(a[0], a[1], 0.0))
			marker.points.append(Point(b[0], b[1], 0.0))

		return marker

	def publish(self, polyline, status):
		self.segments = self.corridor_silhouette(polyline) if len(polyline) >= 2 else []
		self.draw(status)

	def recolour(self, status):
		if self.segments:
			self.draw(status)

	def draw(self, status):
		arr = MarkerArray()
		stamp = rospy.Time.now()

		clear = Marker()
		clear.header.frame_id = self.planning_frame
		clear.header.stamp = stamp
		clear.action = Marker.DELETEALL
		arr.markers.append(clear)

		if self.segments:
			arr.markers.append(self.corridor_marker(self.segments, stamp, status))

		self.markers_pub.publish(arr)