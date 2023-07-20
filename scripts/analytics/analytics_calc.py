#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import sys
import numpy as np
from sensor_msgs.msg import JointState
from analytics_config import *
from analytics_utils import *

# ====================================================================================================================
# Ros Subscriber
# ====================================================================================================================
class AnalyticsNode:
	"""
	ROS Node used to handle parsing data from ROS and recording into text files to be analysed later
	"""
	def __init__(self):
		self.sub = rospy.Subscriber("/os3/step_problem_solution", JointState, self.callback)
		self.angle_array = np.empty((0,3), np.float64)
		self.coord_array = np.empty((0,10), np.float64)
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.default_shoulder_offset = np.array([0.2943, 0.2433, 0.2712])
		print("initialised AnalyticsNode")
		
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

		# trans, rot = self.get_transform()
		# coords[0] += trans

		coords[0] += self.default_shoulder_offset
		coords[0] = np.array([coords[0, 0], coords[0, 2], -coords[0, 1]])
		# print(js.effort[:4])
		# coord_entry = np.append(self.transform_pose(js.velocity[:3], 'panda_link0', 'LShoulder_MidHip_frame'), effort)
		coord_entry = np.append(coords, effort)
		# print(coord_entry)
		coord_entry = np.append(coord_entry, effort_cartesian)
		# print(coord_entry)
		self.coord_array = np.append(self.coord_array, np.array([coord_entry]), axis=0)

		# print(self.tf_buffer.lookup_transform('panda_link0', 'LShoulder_MidHip_frame', rospy.Time(0)))
		print(coord_entry)

		np.savetxt(angle_pipe, self.angle_array)
		np.savetxt(mag_pipe, mag_array)
		np.savetxt(count_pipe, count_array)
		np.savetxt(coord_pipe, self.coord_array)

	def transform_pose(self, input_position, from_frame, to_frame):
		pose_stamped = tf2_geometry_msgs.PoseStamped()
		pose_stamped.pose.position.x = input_position[0]
		pose_stamped.pose.position.y = input_position[1]
		pose_stamped.pose.position.z = input_position[2]
		pose_stamped.pose.orientation.x = 1
		pose_stamped.pose.orientation.y = 0
		pose_stamped.pose.orientation.z = 0
		pose_stamped.pose.orientation.w = 0
		pose_stamped.header.frame_id = from_frame
		pose_stamped.header.stamp = rospy.Time(0)

		try:
			# ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
			output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
			return np.array([[output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z]])

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			raise

	def get_transform(self):
		transform = self.tf_buffer.lookup_transform('panda_link0', 'LShoulder_MidHip_frame', rospy.Time(0))
		translation = transform.transform.translation
		rotation = transform.transform.rotation
		return np.array([translation.x, translation.y, translation.z]), np.array([rotation.x, rotation.y, rotation.z])

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