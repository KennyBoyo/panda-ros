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
		
	def callback(self, js: JointState):
		res = 15

		angles = np.array([js.position]) 
		effort = np.array([js.effort])

		indices, mags = parse_data(angles, effort, res)

		for i in range(len(indices)):
			curr_count = count_array[indices[i][0]-2, indices[i][1]-2]
			curr_mag = mag_array[indices[i][0]-2, indices[i][1]-2]
			mag_array[indices[i][0]-2, indices[i][1]-2] = (curr_mag * curr_count + mags[i]) / (curr_count + 1)
			count_array[indices[i][0]-2, indices[i][1]-2] += 1
			
		np.savetxt(mag_pipe, mag_array)
		np.savetxt(count_pipe, count_array)


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