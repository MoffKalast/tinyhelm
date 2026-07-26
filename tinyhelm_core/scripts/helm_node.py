#!/usr/bin/env python3

from __future__ import annotations

import copy
import math
import rospy
import threading
import pprint
import tf2_ros
import tf

from typing import Dict, Any, Optional

from std_msgs.msg import String, Bool, Empty
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import MarkerArray, Marker

from behaviours import Behaviours, StateAction, Intention
from monitors import Monitors, MonitorAction, MONITOR_STATUS_NAMES
from config_loader import ConfigLoader

from tinyhelm_core.msg import ControllerStatus, MonitorStatus

from util import get_pose_in_frame, strip_repeated_poses, drop_passed_legs

STATUS_NAMES = {
    ControllerStatus.IDLE: "IDLE",
    ControllerStatus.ACTIVE: "ACTIVE",
    ControllerStatus.FINISHED: "FINISHED",
    ControllerStatus.ESTOPPED: "ESTOPPED",
    ControllerStatus.PREEMPTED: "PREEMPTED",
    ControllerStatus.ABORTED: "ABORTED",
    ControllerStatus.ERROR: "ERROR",
}

class HelmCore:
	def __init__(self):
		rospy.init_node("tinyhelm_core")

		# Load parameters under the namespace "tinyhelm_core"
		self.params = rospy.get_param("tinyhelm_core", {})

		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_core'. Did you load the YAML file?")
			raise SystemExit(1)
		
		self.cfg_loader = ConfigLoader(self.params)

		self.ROBOT_FRAME = rospy.get_param("/robot_frame", "base_link")
		self.PLANNING_FRAME = rospy.get_param("/planning_frame", "local")
		
		self.estop_topic = self.params.get("estop_topic", "/tinyhelm/estop")
		self.enabled_topic = self.params.get("enabled_topic", "/tinyhelm/enabled")
		self.markers_topic = self.params.get("markers_topic", "/tinyhelm/markers")
		self.home_topic = self.params.get("home_topic", "/tinyhelm/set_home")

		self.marker_rate = self.params.get("marker_rate", 10.0)
		self.marker_clear_period = self.params.get("marker_clear_period", 5.0)
		self.RATE = rospy.Rate(self.marker_rate)

		# Markers from every source are aggregated into one array and relayed on a fixed tick,
		# keyed by source so a fast publisher replaces its own set instead of stacking duplicates
		self.marker_sources = {}
		self.marker_lock = threading.Lock()
		self.markers_published = False
		self.last_marker_clear = rospy.Time(0)

		self.hold_reach_params = self.params.get("hold_reach_params", [])
		if not self.hold_reach_params:
			rospy.logwarn("No hold_reach_params configured, automatic holds will always stay in place instead of pinning the last waypoint.")

		#Multiplexer for cmd_vel
		cmd_mux_cfg = self.params.get("cmd_vel_mux", {})
		self.selector_topic = cmd_mux_cfg.get("selector_topic", "/cmd_vel_mux/_active_topic")
		self.teleop_topic = cmd_mux_cfg.get("teleop_topic", "/cmd_vel_teleop")

		self.tf2_buffer = tf2_ros.Buffer()
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
		self.selector_pub = rospy.Publisher(self.selector_topic, String, queue_size=1)
		self.markers_pub = rospy.Publisher(self.markers_topic, MarkerArray, queue_size=5)

		self.enabled = False
		self.active_controller: Optional[str] = None
		self.manual_control = False

		# set_intention publishes the plan before recording it, so a controller reporting ACTIVE in that
		# window reaches controller_status_callback before this exists
		self.current_behavior: Optional[Dict[str, Any]] = None

		self.manual_home = None

		self.controllers = self.cfg_loader.parse_controllers(self.controller_status_callback, self.markers_callback, self.current_path_callback)
		self.behaviours = self.cfg_loader.parse_behaviours(self.behaviour_callback)
		self.monitors = self.cfg_loader.parse_monitors(self.monitor_status_callback, self.monitor_revised_path_callback, self.monitor_markers_callback)

		self.estop_sub = rospy.Subscriber(self.estop_topic, Empty, self.estop_callback, queue_size=1)
		self.teleop_override_sub = rospy.Subscriber("/teleop_override_active", Bool, self.teleop_override, queue_size=1)
		self.home_sub =  rospy.Subscriber(self.home_topic, PoseStamped, self.home_callback)
		
		self.enabled_sub = rospy.Subscriber(self.enabled_topic, Bool, self.enabled_callback, queue_size=1)
		self.enabled_pub = rospy.Publisher(self.enabled_topic, Bool, queue_size=1, latch=True)
		self.enabled_pub.publish(self.enabled)

		rospy.loginfo("HelmCore initialized. Waiting for enabled=True to allow behaviours.")

		# Only allow teleop at start
		self.set_cmd_vel_mux("")

	def set_cmd_vel_mux(self, controller_name: str):
		if controller_name in self.controllers and self.enabled:
			rospy.logdebug(f"{controller_name} can now send cmd_vel'")
			self.selector_pub.publish(self.controllers[controller_name]['cmd_vel'])
		else:
			rospy.logdebug(f"Cmd_vel restricted to teleop only.'")
			self.selector_pub.publish("")

	def teleop_override(self, msg: Bool):
		#indicator sent by the cmd_vel_mux, if there's an override happening right now

		if self.manual_control and not msg.data:
			#we just released control
			if self.active_controller == "stationkeeping":
				#move current position hold to new location
				self.behaviour_callback('stationkeeping', None)

		self.manual_control = msg.data
		
	def home_callback(self, msg: PoseStamped):
		self.manual_home = msg

	def enabled_callback(self, msg: Bool):
		if msg.data != self.enabled:
			self.enabled = msg.data
			self.enabled_pub.publish(self.enabled)

			if self.enabled:
				rospy.loginfo(f"Tinyhelm enabled!")
				self.behaviour_callback('stationkeeping', None)
			else:
				rospy.loginfo(f"Tinyhelm disabled!")
				self.set_cmd_vel_mux("")
				self.stop_controller(self.active_controller)
				self.active_controller = None
				self.publish_mission(Path())

	def stop_controller(self, controller: str):
			if controller == self.active_controller:
				self.set_cmd_vel_mux("")
			
			ctrl = self.controllers.get(controller, {})
			stop_pub = ctrl.get('stop_pub')
			if stop_pub:
				stop_pub.publish(Empty())

	def estop_callback(self, msg: Empty):
		if self.active_controller == "stationkeeping":
			rospy.logwarn("Estop triggered, but we're already stopped. Send false to /enable to disable helm entirely.")
			return

		rospy.logwarn("Waypoint ESTOP triggered! Transitioning to stationkeeping.")
		self.behaviour_callback('stationkeeping', None)

	def behaviour_callback(self, behaviour_name: str, msg: Any):
		if not self.enabled:
			rospy.logerr(f"Helm is disabled, ignoring command: {behaviour_name}")
			return

		try:
			get_plan = getattr(Behaviours, behaviour_name)
		except Exception as e:
			rospy.logerr(f"{behaviour_name} not defined. {e}")
			return
		
		if isinstance(msg, Path) and len(msg.poses) == 0:
			rospy.loginfo("Interpreting empty Path as stop.")
			self.behaviour_callback('stationkeeping', None)
			return

		rospy.loginfo(f"Received command for {behaviour_name}, activating controller...")

		#process the plan
		robot_pose = self.wait_for_robot_pose()
		if robot_pose is None:
			return

		intention: Intention = get_plan(robot_pose, msg)

		if intention is None or intention.plan is None:
			rospy.logerr(f"{behaviour_name} returned no plan! What?!")
			return
		
		controller_name = self.behaviours[behaviour_name]['controller']	
		
		# Stop previous controller cleanly before starting new
		if self.active_controller and self.active_controller != controller_name:
			self.stop_controller(self.active_controller)
			rospy.sleep(0.1)
			# Only the outgoing controller's markers go; the aggregate redraws without them on the
			# next tick, so monitor overlays survive the switch
			self.drop_markers(f"controller:{self.active_controller}")

		self.set_intention(intention)

	def wait_for_robot_pose(self) -> Optional[PoseStamped]:
		"""Waits for TF rather than abandoning the plan. A gap here is almost always a transient
		localisation dropout and the vessel is holding position through it anyway, so retrying is
		better than failing a mission over it. Gives up only on shutdown or on being disabled,
		since publishing a plan the operator has since called off would be worse than nothing."""
		while not rospy.is_shutdown():
			if not self.enabled:
				rospy.logwarn("Helm was disabled while waiting for a transform, dropping the plan.")
				return None

			try:
				return get_pose_in_frame(self.tf2_buffer, self.PLANNING_FRAME, self.ROBOT_FRAME)
			except Exception as e:
				rospy.logwarn_throttle(5.0, f"Waiting for {self.PLANNING_FRAME} -> {self.ROBOT_FRAME}: {e}")
				rospy.sleep(0.2)

		return None

	def anchored_plan(self, intention: Intention):
		"""Prepends the vessel's position so the run out to the first waypoint is a real leg of the
		plan, which both keeps the controller from having to invent an anchor of its own and puts
		that transit in front of the monitors, since it is water like any other and may need
		avoiding. Taken fresh on every publish: a RESTART re-sends the same Intention, so an anchor
		captured when a loiter began would by now be a phantom waypoint hours behind us."""
		plan = intention.plan
		if not intention.bootstrap or not isinstance(plan, Path) or not plan.poses:
			return plan

		anchor = self.wait_for_robot_pose()
		if anchor is None:
			return None

		anchored = Path()
		anchored.header = plan.header
		anchored.poses = strip_repeated_poses([anchor] + list(plan.poses))
		return anchored

	def set_intention(self, intention: Intention):

		behaviour_name = intention.name
		controller_name = self.behaviours[behaviour_name]['controller']	
		controller = self.controllers[controller_name]

		plan = self.anchored_plan(intention)
		if plan is None:
			return

		# Let's-a gooooo
		if isinstance(plan, PoseStamped):
			if controller.get('pose_pub'):
				controller['pose_pub'].publish(plan)
			else:
				rospy.logerr(f"{behaviour_name}({controller_name}) cannot receive single goals.")
				return
		elif isinstance(plan, Path):
			if controller.get('path_pub'):
				controller['path_pub'].publish(plan)
			else:
				rospy.logerr(f"{behaviour_name}({controller_name}) cannot receive paths.")
				return
		else:
			rospy.logerr(f"{behaviour_name}({controller_name}) sent incompatible plan object.")
			return

		# store current behaviour metadata. The Intention keeps its original plan so a RESTART
		# re-anchors from scratch; executed_plan is what actually went out
		self.current_behavior = {
			'name': behaviour_name,
			'controller': controller_name,
			'intention': intention,
			'executed_plan': plan
		}
		self.active_controller = controller_name
		self.set_cmd_vel_mux(controller_name)
		self.publish_mission(plan)

	def hold_reach(self):
		"""Summed bound on how far an automatic hold target may sit from the vessel. The params
		are dynamically reconfigurable, so they are read per check rather than cached."""
		reach = 0.0
		for param in self.hold_reach_params:
			value = rospy.get_param(param, None)
			if value is None:
				rospy.logwarn_throttle(30.0, f"Hold reach param {param} is unset, automatic holds will be stricter than intended.")
				continue
			reach += value

		return reach

	def pose_in_planning_frame(self, pose: PoseStamped, fallback_frame: str) -> PoseStamped:
		frame = pose.header.frame_id or fallback_frame
		if not frame or frame == self.PLANNING_FRAME:
			return pose

		transform = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, frame, rospy.Time(0))
		return do_transform_pose(pose, transform)

	def within_hold_reach(self, goal: PoseStamped, fallback_frame: str) -> bool:
		"""A monitor revision can truncate the plan or substitute its last waypoint, so the
		stored plan may end somewhere the vessel never went, possibly inside an obstacle. An
		automatic hold trusts that target only while the vessel is near it. Compared in XY only,
		since both bounds are horizontal and altitude may be ignored entirely."""
		try:
			robot = get_pose_in_frame(self.tf2_buffer, self.PLANNING_FRAME, self.ROBOT_FRAME)
			target = self.pose_in_planning_frame(goal, fallback_frame)
		except Exception as e:
			rospy.logwarn(f"Cannot verify the hold target, holding in place instead: {e}")
			return False

		reach = self.hold_reach()
		distance = math.hypot(
			target.pose.position.x - robot.pose.position.x,
			target.pose.position.y - robot.pose.position.y
		)

		if distance > reach:
			rospy.logwarn(f"Plan finished {distance:.1f}m short of its last waypoint, past the {reach:.1f}m hold reach. The plan was likely truncated, holding in place instead.")
			return False

		return True

	def publish_mission(self, plan):
		"""Mirrors the mission being executed to all monitors; an empty Path means nothing to watch.
		The current path is emptied alongside it, so a monitor cannot keep judging the previous
		mission's course until the controller gets round to publishing its first plan."""
		msg = plan if isinstance(plan, Path) else Path()
		for m in self.monitors.values():
			m['revision_pending'] = False
			m['last_revised_path'] = None
			if m.get('mission_pub'):
				m['mission_pub'].publish(msg)
			if m.get('current_path_pub'):
				m['current_path_pub'].publish(Path())

	def current_path_callback(self, controller_name: str, msg: Path):
		"""Monitors watch what is actually being steered rather than reaching into a controller's
		topics themselves, so the active controller's own plan is relayed on to them here."""
		if controller_name != self.active_controller:
			return

		for m in self.monitors.values():
			if m.get('current_path_pub'):
				m['current_path_pub'].publish(msg)

	def monitor_revised_path_callback(self, monitor_name: str, msg: Path):
		if monitor_name not in self.monitors:
			return
		self.monitors[monitor_name]['last_revised_path'] = msg
		# the REPLAN status can outrun its revision across topics; fulfill the request now
		if self.monitors[monitor_name].get('revision_pending'):
			self.revise_plan(monitor_name)

	def monitor_status_callback(self, monitor_name: str, msg: MonitorStatus):
		rospy.loginfo_throttle(5.0, f"Monitor {monitor_name} reports: {MONITOR_STATUS_NAMES.get(msg.status, '???')} - {msg.message}")

		if not self.enabled:
			return

		action = Monitors.action_for(msg.status)

		if action == MonitorAction.REVISE_PLAN:
			self.revise_plan(monitor_name)
		elif action == MonitorAction.STATIONKEEPING:
			rospy.logwarn(f"Monitor {monitor_name} demands a stop! Transitioning to stationkeeping.")
			self.behaviour_callback('stationkeeping', None)

	def trim_revision(self, revision: Path) -> Path:
		"""Last chance to drop legs the vessel has outrun. A search takes as long as it takes and the
		result then waits on a monitor status before being relayed, by which point its opening leg
		can be well astern, and the controller has no way to tell a stale opening pose from a
		waypoint it is meant to visit."""
		if revision.header.frame_id and revision.header.frame_id != self.PLANNING_FRAME:
			rospy.logwarn_throttle(5.0, f"Revision arrived in {revision.header.frame_id}, not {self.PLANNING_FRAME}, relaying it whole.")
			return revision

		try:
			vessel = get_pose_in_frame(self.tf2_buffer, self.PLANNING_FRAME, self.ROBOT_FRAME)
		except Exception as e:
			rospy.logwarn_throttle(5.0, f"Cannot trim a revision without a transform, relaying it whole: {e}")
			return revision

		trimmed = drop_passed_legs(revision.poses, vessel.pose.position.x, vessel.pose.position.y)
		if len(trimmed) == len(revision.poses):
			return revision

		rospy.loginfo(f"Revision opened {len(revision.poses) - len(trimmed)} leg(s) astern of us, trimmed.")
		out = Path()
		out.header = revision.header
		out.poses = trimmed
		return out

	def revise_plan(self, monitor_name: str):
		revision = self.monitors[monitor_name].get('last_revised_path')
		if revision is None or len(revision.poses) < 2:
			self.monitors[monitor_name]['revision_pending'] = True
			return

		controller = self.controllers.get(self.active_controller, {})
		path_pub = controller.get('path_pub')
		if not path_pub:
			rospy.logwarn_throttle(5.0, f"Monitor {monitor_name} proposed a revision but the active controller cannot accept one.")
			return

		# The controller works out where to rejoin a path by itself, so a revision needs no topic of
		# its own; it is just another path that happens to start next to us
		path_pub.publish(self.trim_revision(revision))
		# consume it so a repeated REPLAN status doesn't re-send an identical revision
		self.monitors[monitor_name]['last_revised_path'] = None
		self.monitors[monitor_name]['revision_pending'] = False

	def controller_status_callback(self, controller_name: str, msg: ControllerStatus):
		rospy.loginfo(f"Controller {controller_name} reports: {STATUS_NAMES.get(msg.status, '???')} - {msg.message}")

		# Case 1: controller goes ACTIVE but helm didn't start it, I guess we're doing that now
		if msg.status == ControllerStatus.ACTIVE and controller_name != self.active_controller:
			rospy.logwarn(f"Controller {controller_name} became active externally. ALRIGHTY THEN!")
			self.active_controller = controller_name
			self.current_behavior = {
				'name': None,
				'controller': controller_name,
				'intention': None
			}

		# If this isn't the active one, ignore
		if controller_name != self.active_controller:
			return

		# Case 2: handle outcomes for the current behaviour
		if not self.current_behavior or not self.current_behavior.get('intention'):
			return

		intention = self.current_behavior['intention']

		if msg.status == ControllerStatus.FINISHED:
			if intention.on_finish == StateAction.HOLD_POSITION:
				rospy.loginfo("Finished plan, switching to stationkeeping.")
				executed = self.current_behavior.get('executed_plan')

				if isinstance(executed, Path) and executed.poses and self.within_hold_reach(executed.poses[-1], executed.header.frame_id):
					self.behaviour_callback('stationkeeping', executed.poses[-1]) #stop at last waypoint exactly
				else:
					self.behaviour_callback('stationkeeping', None)# just stay where you are

			elif intention.on_finish == StateAction.RESTART:
				rospy.loginfo("Finished plan, restarting...")
				self.set_intention(intention)

			elif intention.on_finish == StateAction.IDLE:
				rospy.loginfo("Finished plan, going IDLE.")
				self.active_controller = None
				self.current_behavior = None
				self.set_cmd_vel_mux("")

			elif intention.on_finish == StateAction.RETURN_TO_HOME:
				rospy.loginfo("Finished plan, returning home #TODO.")

		elif msg.status in (ControllerStatus.ABORTED, ControllerStatus.ERROR):
			rospy.logwarn(f"Controller {controller_name} is not feeling well: {msg.message}")
			self.behaviour_callback('stationkeeping', None)

	def markers_callback(self, controller_name, msg: MarkerArray):
		if controller_name != self.active_controller:
			self.drop_markers(f"controller:{controller_name}")
			return

		self.store_markers(f"controller:{controller_name}", msg)

	def monitor_markers_callback(self, monitor_name: str, msg: MarkerArray):
		# Monitors are relayed whoever is driving: an obstacle overlay is most wanted precisely
		# when a monitor has forced the helm into stationkeeping
		self.store_markers(f"monitor:{monitor_name}", msg)

	def store_markers(self, source: str, msg: MarkerArray):
		"""A source clearing its own markers must never reach the aggregate, since one DELETEALL
		in there wipes every other source's namespaces too. It drops the source's entry instead,
		and the leading wipe in relay_markers takes care of the rest."""
		delete_all = getattr(Marker, "DELETEALL", 3)
		markers = [m for m in msg.markers if m.action != delete_all]

		if not markers:
			self.drop_markers(source)
			return

		with self.marker_lock:
			self.marker_sources[source] = markers

	def drop_markers(self, source: str):
		with self.marker_lock:
			self.marker_sources.pop(source, None)

	def clear_markers(self):
		m = Marker()
		m.action = getattr(Marker, "DELETEALL", 3)
		ma = MarkerArray()
		ma.markers.append(m)
		self.markers_pub.publish(ma)

	def relay_markers(self):
		if not self.enabled:
			# Nothing should be driving, so wipe the display, but slowly enough not to spam it
			if (rospy.Time.now() - self.last_marker_clear).to_sec() >= self.marker_clear_period:
				self.clear_markers()
				self.last_marker_clear = rospy.Time.now()
				self.markers_published = False
				with self.marker_lock:
					self.marker_sources.clear()
			return

		with self.marker_lock:
			sources = list(self.marker_sources.values())

		if not sources:
			# One final wipe on the transition, then stay quiet
			if self.markers_published:
				self.clear_markers()
				self.markers_published = False
			return

		# A leading wipe followed by a full redraw in the same array: a source that stopped
		# publishing disappears without the helm tracking every namespace and id it ever relayed.
		# Rviz applies the whole array in one pass, so this does not flicker.
		arr = MarkerArray()
		arr.markers.append(Marker())
		arr.markers[0].action = getattr(Marker, "DELETEALL", 3)
		for markers in sources:
			arr.markers.extend(markers)

		self.markers_pub.publish(arr)
		self.markers_published = True

	def update(self):
		self.relay_markers()

if __name__ == "__main__":
	try:
		node = HelmCore()
		while not rospy.is_shutdown():
			try:
				node.RATE.sleep()
			except rospy.ROSTimeMovedBackwardsException:
				rospy.logwarn("Time moved backwards, resetting the marker relay clock.")
				node.RATE = rospy.Rate(node.marker_rate)
				continue
			except rospy.ROSInterruptException:
				break
			node.update()
	except rospy.ROSInterruptException:
		rospy.logwarn("Tinyhelm shutting down...")
		pass
