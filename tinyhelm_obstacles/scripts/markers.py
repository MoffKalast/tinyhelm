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

		# Kept so a change of status can recolour what is already on screen without recomputing the
		# union. The status arrives after the redraw that prompted
		# it, so without this the colour would always be one cycle behind what the vessel is doing.
		self.segments = []

	def corridor_silhouette(self, polyline):
		"""Boundary of the union of the leg tubes, as unordered pairs of endpoints. Buffering the
		polyline produces the same stadium chain the planner builds capsule by capsule, joins included,
		so the union is one call rather than a stamp per leg.

		The tube is the whole fence, so this is the whole fence: there is nothing anchored to the vessel
		to add, and what is drawn is what a search will be held to.

		Pairs rather than a ring because a LINE_LIST needs no ordering, and so needs no case for the
		gaps a pattern that doubles back encloses between its own legs."""
		union = unary_union([LineString(polyline).buffer(self.max_detour, resolution=8)])

		segments = []
		for geom in getattr(union, "geoms", (union,)):
			for ring in [geom.exterior] + list(geom.interiors):
				coords = list(ring.coords)
				segments.extend((coords[i - 1], coords[i]) for i in range(1, len(coords)))

		return segments

	def corridor_marker(self, segments, stamp, status):
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
		marker.color.r, marker.color.g, marker.color.b = STATUS_COLOURS.get(status, STATUS_COLOURS[MonitorStatus.OK])
		marker.color.a = 0.7

		for a, b in segments:
			marker.points.append(Point(a[0], a[1], 0.0))
			marker.points.append(Point(b[0], b[1], 0.0))

		return marker

	def publish(self, polyline, status):
		"""One outline for the whole allowed region rather than a stadium per leg. Stacked stadiums are
		unreadable on a survey pattern, where every leg overlaps its neighbours and the interior fills
		with the flanks of tubes that are not the boundary of anything."""
		self.segments = self.corridor_silhouette(polyline) if len(polyline) >= 2 else []
		self.draw(status)

	def recolour(self, status):
		"""Redraws what is already on screen in the colour of a status that has just changed. Cheap: the
		union is the expensive part and it has not moved."""
		if self.segments:
			self.draw(status)

	def draw(self, status):
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

		if self.segments:
			arr.markers.append(self.corridor_marker(self.segments, stamp, status))

		self.markers_pub.publish(arr)