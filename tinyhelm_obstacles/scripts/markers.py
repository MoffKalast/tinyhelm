#!/usr/bin/env python3
import rospy

from shapely.geometry import LineString
from shapely.geometry import Point as ShapelyPoint
from shapely.ops import unary_union
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import Point

class DebugMarkers:

	def __init__(self, planning_frame, max_detour):
		self.planning_frame = planning_frame
		self.max_detour = max_detour
		self.markers_pub = rospy.Publisher("/obstacles/_markers", MarkerArray, queue_size=1, latch=True)

	def corridor_silhouette(self, polyline, position):
		"""Boundary of the union of the leg tubes and the disc at the vessel, as unordered pairs of
		endpoints. Buffering the polyline produces the same stadium chain the planner builds capsule by
		capsule, joins included, so the union is one call rather than a stamp per leg.

		Pairs rather than a ring because a LINE_LIST needs no ordering, and so needs no case for either
		of the two things a pattern that doubles back produces: gaps enclosed between legs, and a piece
		detached from the rest. The detached case is worth seeing rather than smoothing away, since a
		vessel whose disc no longer touches the tube has nowhere legal for a correction to begin."""
		shapes = [LineString(polyline).buffer(self.max_detour, resolution=8)]
		if position is not None:
			shapes.append(ShapelyPoint(position[0], position[1]).buffer(self.max_detour, resolution=8))

		union = unary_union(shapes)

		segments = []
		for geom in getattr(union, "geoms", (union,)):
			for ring in [geom.exterior] + list(geom.interiors):
				coords = list(ring.coords)
				segments.extend((coords[i - 1], coords[i]) for i in range(1, len(coords)))

		return segments

	def corridor_marker(self, segments, stamp):
		"""The boundary the search is actually held to, so it doubles as a picture of how much space a
		correction has to work in."""
		marker = Marker()
		marker.header.frame_id = self.planning_frame
		marker.header.stamp = stamp
		marker.ns = "corridor"
		marker.id = 0
		marker.type = Marker.LINE_LIST
		marker.action = Marker.ADD
		marker.scale.x = 0.2
		marker.pose.orientation.w = 1.0
		marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.6, 1.0, 0.7

		for a, b in segments:
			marker.points.append(Point(a[0], a[1], 0.0))
			marker.points.append(Point(b[0], b[1], 0.0))

		return marker

	def publish(self, polyline, position):
		"""One outline for the whole allowed region rather than a stadium per leg. Stacked stadiums are
		unreadable on a survey pattern, where every leg overlaps its neighbours and the interior fills
		with the flanks of tubes that are not the boundary of anything."""
		arr = MarkerArray()
		stamp = rospy.Time.now()

		# The old drawing published one marker id per leg, and rviz keeps anything it is not told to
		# forget. Without this a mission with fewer legs than the last one leaves the surplus stadiums
		# on screen for good, which looks exactly like the pile-up this replaces.
		clear = Marker()
		clear.header.frame_id = self.planning_frame
		clear.header.stamp = stamp
		clear.action = Marker.DELETEALL
		arr.markers.append(clear)

		if len(polyline) >= 2:
			segments = self.corridor_silhouette(polyline, position)
			arr.markers.append(self.corridor_marker(segments, stamp))

		self.markers_pub.publish(arr)