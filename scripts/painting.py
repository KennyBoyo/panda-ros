#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
import numpy as np
from std_msgs.msg import String, Int8, ColorRGBA
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
		self.pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/painting", Marker, queue_size=5)

		self.robot_pose_eq = PoseStamped()
		self.robot_pose = PoseStamped()
		
		self.robot_pose_list = []

		self.robot_pose.header.frame_id = "panda_link0"

		
		self.pos_marker = Marker()

		# Resolution of cube (10^(-cube_res))
		self.cube_res_places = 1
		self.cube_res = 10**(-self.cube_res_places)

		# Side width of cube workspace (meters) (min is 2x max reach of robot)
		self.cube_len = 2
		self.cube_grid = np.zeros((int(self.cube_len/self.cube_res), int(self.cube_len/self.cube_res), int(self.cube_len/self.cube_res)), dtype=np.bool8)

		# Unique Marker ID
		self.idx = 0


	def paint(self, state: FrankaState):
		self.init_marker()
		self.generate_marker(state)

	def init_marker(self):
		# force_quat = tr.quaternion_from_matrix(T_force)
		self.pos_marker.id = self.idx
		# self.idx += 1
		self.pos_marker.header.frame_id = self.robot_pose.header.frame_id
		# force_quat /= np.linalg.norm(force_quat)
		self.pos_marker.pose.orientation.x = 0
		self.pos_marker.pose.orientation.y = 0
		self.pos_marker.pose.orientation.z = 0
		self.pos_marker.pose.orientation.w = 1
		self.pos_marker.color.a = 1
		self.pos_marker.color.r = 1.0
		self.pos_marker.color.g = 0.0
		self.pos_marker.color.b = 0.0

		self.pos_marker.type = Marker.CUBE_LIST
		self.pos_marker.header.stamp = rospy.Time.now()
		self.pos_marker.scale.x = self.cube_res
		self.pos_marker.scale.y = self.cube_res
		self.pos_marker.scale.z = self.cube_res

	def generate_marker(self, state):
		x, y, z = self.snap_grid(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14])

		taken = self.check_point(x, y, z)
		if not taken:
			return
		
		point = Point()
		point.x = x
		point.y = y
		point.z = z
		
		colour = ColorRGBA()
		colour.a = 0.1
		colour.r = 1
		colour.g = 0
		colour.b = 0

		self.pos_marker
		self.pos_marker.points.append(point)
		self.pos_marker.colors.append(colour)

		print(self.pos_marker)

		# self.pos_markers.markers.append(pos_marker)
		self.pub.publish(self.pos_marker)


	def check_point(self, x, y, z):
		cube_grid_index_offset = self.cube_len/2
		x, y, z = int((cube_grid_index_offset + x)/self.cube_res), int((cube_grid_index_offset + y)/self.cube_res), int((cube_grid_index_offset + z)/self.cube_res)
		
		if not self.cube_grid[x,y,z]:
			self.cube_grid[x,y,z] = 1
			return True
		return False


	def snap_grid(self, x, y, z):
		return np.around(x, self.cube_res_places), np.around(y, self.cube_res_places), np.around(z, self.cube_res_places)
		
def main(args):
	rospy.init_node('paint_publisher', anonymous=True, log_level=rospy.DEBUG)
	obc = paint_publisher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)