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

from behaviours import Behaviours
from config_loader import ConfigLoader
from state_machine import StateMachine, HelmState

from tinyhelm_core.msg import ControllerStatus

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
		self.smach = StateMachine()

		self.ROBOT_FRAME = self.params.get("robot_frame", "base_link")
		self.PLANNING_FRAME = self.params.get("planning_frame", "map")
		self.estop_topic = self.params.get("estop_topic", "/tinyhelm/estop")
		self.enabled_topic = self.params.get("enabled_topic", "/tinyhelm/enabled")
		self.markers_topic = self.params.get("markers_topic", "/tinyhelm/markers")

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

		self.controllers = self.cfg_loader.parse_controllers(self.controller_status_callback, self.markers_callback)
		self.behaviours = self.cfg_loader.parse_behaviours(self.behaviour_callback)

		self.estop_sub = rospy.Subscriber(self.estop_topic, Empty, self.estop_callback, queue_size=1)
		
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
		
	def enabled_callback(self, msg: Bool):
		if msg.data != self.enabled:
			self.enabled = msg.data
			self.enabled_pub.publish(self.enabled)

			if self.enabled:
				rospy.loginfo(f"Tinyhelm enabled!")
			else:
				rospy.loginfo(f"Tinyhelm disabled!")
				self.set_cmd_vel_mux("")
				self.stop_controller(self.active_controller)
				self.active_controller = None

	def stop_controller(self, controller: str):
			ctrl = self.controllers.get(controller, {})
			stop_pub = ctrl.get('stop_pub')
			if stop_pub:
				stop_pub.publish(Empty())

	def estop_callback(self, msg: Empty):
		if self.active_controller == "stationkeeping":
			rospy.logwarn("Estop triggered, but we're already stopped. Send false to /enable to disable helm entirely.")
			return

		rospy.logwarn("Waypoint ESTOP triggered! Transitioning to stationkeeping.")

		self.set_cmd_vel_mux("")
		self.stop_controller(self.active_controller)

		rospy.loginfo(f"Stopped '{self.active_controller}.'")
		self.active_controller = None

		#TODO stationkeeping transition

	def behaviour_callback(self, behaviour_name: str, msg: Any):
		if not self.enabled:
			rospy.logerr(f"Helm is disabled, ignoring command: {behaviour_name}")
			return
		
		try:
			get_plan = getattr(Behaviours, behaviour_name)
		except Exception as e:
			rospy.logerr(f"{behaviour_name} not defined. {e}")
			return
		
		rospy.loginfo(f"Received command for {behaviour_name}, activating controller...")

		#find the behaviour's nav controller
		controller_name = self.behaviours[behaviour_name]['controller']	
		controller = self.controllers[controller_name]

		#process the plan
		robot_pose = self.tf2_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0))
		nav_plan = get_plan(robot_pose, msg)

		if type(nav_plan) == PoseStamped:
			if controller['pose_topic']:
				controller['pose_pub'].publish(nav_plan)
				rospy.loginfo(f"Published plan: {nav_plan}")
			else:
				rospy.logerr(f"{behaviour_name}({controller}) cannot receive single goals.")
				return
		elif type(nav_plan) == Path:
			if controller['path_topic']:
				controller['path_pub'].publish(nav_plan)
				rospy.loginfo(f"Published plan: {nav_plan}")
			else:
				rospy.logerr(f"{behaviour_name}({controller}) cannot receive paths.")
				return
		
		self.active_controller = controller_name
		self.set_cmd_vel_mux(controller_name)

	def controller_status_callback(self, controller_name, msg):
		rospy.loginfo(f"Controller {controller_name} reports:{STATUS_NAMES.get(msg.status)}, {msg.message}")

		if controller_name == self.active_controller:
			if msg.status == ControllerStatus.ESTOPPED:
				pass


		#TODO monitor if active controller is enabled, do recovery things if not
		pass

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

		rospy.loginfo(f"Active controller {self.active_controller}")

if __name__ == "__main__":
	try:
		node = HelmCore()
		rate = rospy.Rate(0.5)
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
