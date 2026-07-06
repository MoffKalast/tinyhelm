#!/usr/bin/env python3

from __future__ import annotations

import copy
import rospy
import pprint
import tf2_ros
import tf

from typing import Dict, Any, Optional

from std_msgs.msg import String, Bool, Empty
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

from behaviours import Behaviours, StateAction, Intention
from config_loader import ConfigLoader

from tinyhelm_core.msg import ControllerStatus

from util import get_pose_in_frame

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
		rospy.init_node("tinyhelm_core_node")

		# Load parameters under the namespace "tinyhelm_core"
		self.params = rospy.get_param("tinyhelm_core", {})

		if not self.params:
			rospy.logwarn("No parameters found under 'tinyhelm_core'. Did you load the YAML file?")
			raise SystemExit(1)
		
		self.cfg_loader = ConfigLoader(self.params)

		self.ROBOT_FRAME = rospy.get_param("/robot_frame", "base_link")
		self.PLANNING_FRAME = rospy.get_param("/planning_frame", "map")
		
		self.estop_topic = self.params.get("estop_topic", "/tinyhelm/estop")
		self.enabled_topic = self.params.get("enabled_topic", "/tinyhelm/enabled")
		self.markers_topic = self.params.get("markers_topic", "/tinyhelm/markers")
		self.home_topic = self.params.get("home_topic", "/tinyhelm/set_home")

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

		self.manual_home = None

		self.controllers = self.cfg_loader.parse_controllers(self.controller_status_callback, self.markers_callback)
		self.behaviours = self.cfg_loader.parse_behaviours(self.behaviour_callback)

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
		robot_pose: PoseStamped = get_pose_in_frame(self.tf2_buffer, self.PLANNING_FRAME, self.ROBOT_FRAME)
		intention: Intention = get_plan(robot_pose, msg)

		if intention is None or intention.plan is None:
			rospy.logerr(f"{behaviour_name} returned no plan! What?!")
			return
		
		controller_name = self.behaviours[behaviour_name]['controller']	
		
		# Stop previous controller cleanly before starting new
		if self.active_controller and self.active_controller != controller_name:
			self.stop_controller(self.active_controller)
			rospy.sleep(0.1)
			self.clear_markers()

		self.set_intention(intention)

	def set_intention(self, intention: Intention):

		behaviour_name = intention.name
		controller_name = self.behaviours[behaviour_name]['controller']	
		controller = self.controllers[controller_name]

		# Let's-a gooooo
		if isinstance(intention.plan, PoseStamped):
			if controller.get('pose_pub'):
				controller['pose_pub'].publish(intention.plan)
			else:
				rospy.logerr(f"{behaviour_name}({controller_name}) cannot receive single goals.")
				return
		elif isinstance(intention.plan, Path):
			if controller.get('path_pub'):
				controller['path_pub'].publish(intention.plan)
			else:
				rospy.logerr(f"{behaviour_name}({controller_name}) cannot receive paths.")
				return
		else:
			rospy.logerr(f"{behaviour_name}({controller_name}) sent incompatible plan object.")
			return

		# store current behaviour metadata
		self.current_behavior = {
			'name': behaviour_name,
			'controller': controller_name,
			'intention': intention
		}
		self.active_controller = controller_name
		self.set_cmd_vel_mux(controller_name)

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

				if isinstance(intention.plan, Path):
					self.behaviour_callback('stationkeeping', intention.plan.poses[-1]) #stop at last waypoint exactly
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

	def markers_callback(self, controller_name, msg):
		if controller_name == self.active_controller:
			self.markers_pub.publish(msg)

	def clear_markers(self):
		m = Marker()
		m.action = getattr(Marker, "DELETEALL", 3)
		ma = MarkerArray()
		ma.markers.append(m)
		self.markers_pub.publish(ma)

	def update(self):
		if self.active_controller is None:
			self.clear_markers()

if __name__ == "__main__":
	try:
		node = HelmCore()
		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			#try:
			node.update()
			#except Exception as e:
				#rospy.logwarn(str(e))
				#return
			rate.sleep()
	except rospy.ROSInterruptException:
		rospy.logwarn("Tinyhelm shutting down...")
		pass
