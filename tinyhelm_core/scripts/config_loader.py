import rospy
import pprint

from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tinyhelm_core.msg import ControllerStatus

class ConfigLoader:
	
	def __init__(self, params):
		self.params = params
		rospy.logdebug("Loaded parameters for tinyhelm_core:")
		rospy.logdebug(pprint.pformat(self.params))

	def parse_controllers(self, controller_status_callback, markers_callback):
		controllers = {}
		ctrl_params = self.params.get("controllers", {})
		for name, cfg in ctrl_params.items():
			c = {}
			c['path_topic'] = cfg.get('path')         # Path (nav_msgs/Path)
			c['pose_topic'] = cfg.get('pose')         # PoseStamped
			c['stop_topic'] = cfg.get('stop')         # std_msgs/Empty pub
			c['cmd_vel'] = cfg.get('cmd_vel')         # string topic for mux selector
			c['status_topic'] = cfg.get('status')   # bool topic that reports controller healthy
			c['markers_topic'] = cfg.get('markers')   # MarkerArray emitted by controller

			if c['path_topic']:
				c['path_pub'] = rospy.Publisher(c['path_topic'], Path, queue_size=1, latch=False)
				rospy.loginfo(f"Controller[{name}] will publish path -> {c['path_topic']}")
			
			if c['pose_topic']:
				c['pose_pub'] = rospy.Publisher(c['pose_topic'], PoseStamped, queue_size=1, latch=False)
				rospy.loginfo(f"Controller[{name}] will publish pose -> {c['pose_topic']}")
			
			if c['stop_topic']:
				c['stop_pub'] = rospy.Publisher(c['stop_topic'], Empty, queue_size=1, latch=False)
				rospy.loginfo(f"Controller[{name}] stop publisher -> {c['stop_topic']}")
			else:
				rospy.logerr(f"Controller[{name}] is missing a stop topic!")

			if c['status_topic']:
				c['status_sub'] = rospy.Subscriber(c['status_topic'], ControllerStatus, lambda msg, nm=name: controller_status_callback(nm, msg), queue_size=1)
				rospy.loginfo(f"Controller[{name}] status monitor -> {c['status_topic']}")
			else:
				rospy.logerr(f"Controller[{name}] is missing a status topic!")

			if c['markers_topic']:
				c['markers_sub'] = rospy.Subscriber(c['markers_topic'], MarkerArray, lambda msg, nm=name: markers_callback(nm, msg), queue_size=1)
				rospy.loginfo(f"Controller[{name}] markers -> {c['markers_topic']}")
			else:
				rospy.logwarn(f"Controller[{name}] does not have a marker topic?")

			controllers[name] = c
			
		return controllers

	def parse_behaviours(self, behaviour_callback):
		behaviours = {}
		for name, cfg in self.params.get("behaviour_topics", {}).items():
			topic = cfg.get('topic')
			controller = cfg.get('controller')
			type = cfg.get('type')

			if not topic or not controller:
				rospy.logwarn(f"Behaviour {name} missing topic/controller - skipping")
				continue

			if type == "PoseStamped":
				sub = rospy.Subscriber(
					topic, 
					PoseStamped,
					lambda msg, 
					bn=name: behaviour_callback(bn, msg),
					queue_size=1
				)
			elif type == "Path":
				sub = rospy.Subscriber(
					topic, 
					Path,
					lambda msg, 
					bn=name: behaviour_callback(bn, msg),
					queue_size=1
				)

			behaviours[name] = {
				'topic': topic,
				'controller': controller,
				'subscriber': sub
			}

			rospy.loginfo(f"Subscribed behaviour '{name}' (Topic) -> {topic} ({type}) -> controller '{controller}'")
			
		return behaviours