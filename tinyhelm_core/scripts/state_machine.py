from enum import Enum, auto

class HelmState(Enum):
    IDLE = auto()
    WAYPOINTS = auto()
    STATIONKEEPING = auto()
    LOITERING = auto()

class StateMachine:
    def __init__(self):
        self.state = HelmState.IDLE

    """def transition(self, new_state: HelmState):
        if self.can_transition(new_state):
            print(f"Transition: {self.state.name} -> {new_state.name}")
            self.state = new_state
        else:
            print(f"Invalid transition: {self.state.name} -> {new_state.name}")

	def _transition_to_stationkeeping(self):
		sk = self.controllers.get('stationkeeping')
		if not sk:
			rospy.logwarn("No stationkeeping controller configured; cannot transition.")
			return

		# lookup transform planning_frame <- robot_frame to get robot pose in planning_frame
		try:
			trans = self.tf_buffer.lookup_transform(self.PLANNING_FRAME, self.ROBOT_FRAME, rospy.Time(0), rospy.Duration(1.0))
			# build PoseStamped
			ps = PoseStamped()
			ps.header.stamp = rospy.Time.now()
			ps.header.frame_id = self.PLANNING_FRAME
			ps.pose.position.x = trans.transform.translation.x
			ps.pose.position.y = trans.transform.translation.y
			ps.pose.position.z = trans.transform.translation.z
			ps.pose.orientation = trans.transform.rotation
			# publish to stationkeeping pose topic
			pose_pub = sk.get('pose_pub')
			if pose_pub:
				pose_pub.publish(ps)
				rospy.loginfo("Published stationkeeping hold-position with current robot pose.")
			else:
				rospy.logwarn("Stationkeeping controller has no pose publisher configured.")
		except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, Exception) as e:
			rospy.logwarn(f"Could not lookup transform {self.PLANNING_FRAME} <- {self.ROBOT_FRAME}: {e}")
			rospy.logwarn("Stationkeeping transition aborted.")
			return

		# Activate stationkeeping controller (set selector + relay markers)
		cmd_vel_topic = sk.get('cmd_vel') or ""
		self._activate_controller('stationkeeping', cmd_vel_topic)"""