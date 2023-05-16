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
		self.angle_array = np.empty((0,3), np.float64)
		self.coord_array = np.empty((0,10), np.float64)
		
	def callback(self, js: JointState):
		res = 15

		angles = np.array([js.position[:4]]) 
		coords = np.array([js.velocity[:3]])
		effort = np.array([js.effort[:4]])

		effort_cartesian = np.array([js.effort[4:7]])

		indices, mags = parse_data(angles, effort, res)

		for i in range(len(indices)):
			curr_count = count_array[indices[i][0]-2, indices[i][1]-2]
			curr_mag = mag_array[indices[i][0]-2, indices[i][1]-2]
			mag_array[indices[i][0]-2, indices[i][1]-2] = (curr_mag * curr_count + mags[i]) / (curr_count + 1)
			count_array[indices[i][0]-2, indices[i][1]-2] += 1
			
		self.angle_array = np.append(self.angle_array, np.array([shoulder2cartesian(angles[0])]), axis = 0)

		print(js.effort[:4])
		coord_entry = np.append(coords, effort)
		print(coord_entry)
		coord_entry = np.append(coord_entry, effort_cartesian)
		print(coord_entry)
		self.coord_array = np.append(self.coord_array, np.array([coord_entry]), axis=0)

		np.savetxt(angle_pipe, self.angle_array)
		np.savetxt(mag_pipe, mag_array)
		np.savetxt(count_pipe, count_array)
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