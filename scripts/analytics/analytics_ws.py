#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from sensor_msgs.msg import JointState
from analytics_config import *
from analytics_utils import *

# ====================================================================================================================
# Ros Subscriber
# ====================================================================================================================
class AnalyticsNode:
	def __init__(self):
		self.sub = rospy.Subscriber("/os3/step_problem_solution", JointState, self.callback)
		self.coord_array = np.empty((0,3), np.float64)
		
	def callback(self, js: JointState):
		angles = np.array([js.position])
		self.coord_array = np.append(self.coord_array, np.array([shoulder2cartesian(angles[0])]), axis = 0)
		np.savetxt(coord_pipe, self.coord_array)

# ====================================================================================================================
# Main
# ====================================================================================================================
def main(args):
	rospy.init_node("analytics_node", anonymous=True)
	an = AnalyticsNode()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)