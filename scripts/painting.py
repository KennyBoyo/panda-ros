#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
import numpy as np
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PointStamped, PoseStamped, Point, WrenchStamped
from dynamic_reconfigure.msg import Config, DoubleParameter, GroupState
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import MarkerArray, Marker
from franka_msgs.msg import FrankaState
import tf.transformations as tr
from panda_ros.msg import ImpedanceParams, StiffnessConfig
from copy import deepcopy

class paint_publisher:

	def __init__(self):
		self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.paint)
		self.pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/painting", MarkerArray, queue_size=5)

		self.robot_pose_eq = PoseStamped()
		self.robot_pose = PoseStamped()
		
		self.robot_pose_list = []

		self.robot_pose.header.frame_id = "panda_link0"

		self.cube_res = 0.01
		self.cubelist = []


	def paint(self, state: FrankaState):

		self.robot_pose.header.stamp = rospy.Time.now()
		
		self.robot_pose.pose.position.x = state.O_T_EE[12]

		self.robot_pose.pose.position.y = state.O_T_EE[13]

		self.robot_pose.pose.position.z = state.O_T_EE[14]

		self.generate_marker(state)

	def generate_marker(self, state):
		pos_markers = MarkerArray()
		pos_marker = Marker()
		

		# If force or wrench is exactly zero, we are approaching joint limit so don't do anything
		if (f_mag == 0 or t_mag == 0):
			return

		
		# force_quat = tr.quaternion_from_matrix(T_force)
		pos_marker.id = 23
		pos_marker.header.frame_id = self.robot_pose.header.frame_id
		pos_marker.pose.position.x = state.O_T_EE[12]
		pos_marker.pose.position.y = state.O_T_EE[13]
		pos_marker.pose.position.z = state.O_T_EE[14]
		# force_quat /= np.linalg.norm(force_quat)
		# force_marker.pose.orientation.x = force_quat[0]
		# force_marker.pose.orientation.y = force_quat[1]
		# force_marker.pose.orientation.z = force_quat[2]
		# force_marker.pose.orientation.w = force_quat[3]
		pos_marker.color.a = 0.5
		pos_marker.color.r = 1.0
		pos_marker.color.g = 0.0
		pos_marker.color.b = 0.0

		pos_marker.type = Marker.CUBE_LIST
		pos_marker.header.stamp = rospy.Time.now()
		pos_marker.scale.x = self.cube_res
		pos_marker.scale.y = 0
		pos_marker.scale.z = 0
		
		pos_markers.markers.append(pos_marker)
		self.pub.publish(pos_markers)

		
def main(args):
	rospy.init_node('paint_publisher', anonymous=True, log_level=rospy.DEBUG)
	obc = paint_publisher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)