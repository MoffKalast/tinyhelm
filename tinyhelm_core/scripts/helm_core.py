#!/usr/bin/env python3
import rospy

class HelmCore:
	def __init__(self):
		rospy.init_node("tinyhelm_core_node")

	def run(self):
		rospy.spin()

if __name__ == "__main__":
	try:
		node = HelmCore()
		node.run()
	except rospy.ROSInterruptException:
		pass