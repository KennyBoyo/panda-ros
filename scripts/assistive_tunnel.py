#!/usr/bin/env python3
import rospy 
import sys
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from dynamic_reconfigure.msg import DoubleParameter
from nav_msgs.msg import Path
from panda_ros.msg import ImpedanceParams
from franka_msgs.msg import FrankaState
import tf.transformations as tr
from copy import deepcopy
import numpy as np
import tf2_ros

# X, Y and Z elements in Float64 Tmatrix (FrankaState)
px = 12
py = 13
pz = 14


def transformation_matrix_to_PoseStamped(trans_mat, frame_id: str) -> PoseStamped:
	msg = PoseStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = frame_id
	# Convert robot end effector coordinates to quaternion
	quat = tr.quaternion_from_matrix([  [trans_mat[0], trans_mat[4], trans_mat[8], trans_mat[12]], \
							[trans_mat[1], trans_mat[5], trans_mat[9], trans_mat[13]], \
						[trans_mat[2], trans_mat[6], trans_mat[10], trans_mat[14]],\
							[trans_mat[3], trans_mat[7], trans_mat[11], trans_mat[15]]])
	# Set end effector orientation
	msg.pose.orientation.x = quat[0]
	msg.pose.orientation.y = quat[1]
	msg.pose.orientation.z = quat[2]
	msg.pose.orientation.w = quat[3]
	
	# Set end effector robot position
	msg.pose.position.x = trans_mat[px]
	msg.pose.position.y = trans_mat[py]
	msg.pose.position.z = trans_mat[pz]

	return msg

def transformation_matrix_to_TransformStamped(trans_mat, frame_id: str, child_id: str) -> TransformStamped:
	msg = TransformStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = frame_id
	msg.child_frame_id = child_id

	# Convert robot end effector coordinates to quaternion
	quat = tr.quaternion_from_matrix([  [trans_mat[0], trans_mat[4], trans_mat[8], trans_mat[12]], \
							[trans_mat[1], trans_mat[5], trans_mat[9], trans_mat[13]], \
						[trans_mat[2], trans_mat[6], trans_mat[10], trans_mat[14]],\
							[trans_mat[3], trans_mat[7], trans_mat[11], trans_mat[15]]])
	# Set end effector orientation
	msg.transform.rotation.x = quat[0]
	msg.transform.rotation.y = quat[1]
	msg.transform.rotation.z = quat[2]
	msg.transform.rotation.w = quat[3]
	
	# Set end effector robot position
	msg.transform.translation.x = trans_mat[px]
	msg.transform.translation.y = trans_mat[py]
	msg.transform.translation.z = trans_mat[pz]

	return msg

class AssistiveTunnelController():
	def __init__(self, tunnelRadius: np.double):
		self.tunnelRadius = tunnelRadius
		self.initialisedTrajectory = False 

		self.end_effector_position_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.step )
		
		self.equilibrium_position_publisher = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped)
		self.stiffness_matrix_publisher = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_stiffness", ImpedanceParams)
		self.trajectory_publisher = rospy.Publisher("/assistance_tunnel/desired_trajectory", Path, queue_size=10)

	def trajectory_generator(self, x0, y0, z0, n):
		traj_position = np.zeros((n, 3))
		traj_position[:, 2] = z0

		n_step = np.linspace(0, 0.1*np.pi, n)
		traj_position[:, 0] = x0 + n_step
		traj_position[:, 1] = 0.1*np.sin(10*n_step) + y0

		return traj_position

	def initialise_trajectory(self, t0: list):

		if not self.initialisedTrajectory:
			trans_mat = t0

			originalPose = transformation_matrix_to_PoseStamped(trans_mat, "panda_link0")

			# Initialise Trajectory
			self.trajectory = self.trajectory_generator(x0 = trans_mat[px],\
														y0 = trans_mat[py],\
														z0 = trans_mat[pz],\
														n = 30)
			# Create nav_msgs Path
			self.pathMsg = Path()
			self.pathMsg.header.stamp = rospy.Time.now()
			self.pathMsg.header.frame_id = "panda_link0"

			for coord in self.trajectory:
				new_pose = deepcopy(originalPose)
				new_pose.pose.position.x = coord[0]
				new_pose.pose.position.y = coord[1]
				new_pose.pose.position.z = coord[2]
				new_pose.header.stamp = rospy.Time.now()
				self.pathMsg.poses.append(new_pose)
	
	def step(self, msg: FrankaState):
		trans_mat = msg.O_T_EE

		# Generate trajectory (only run once)
		if not self.initialisedTrajectory:
			self.initialise_trajectory(trans_mat)
			self.initialisedTrajectory = True

		# Publish trajectory for visualisation
		self.pathMsg.header.stamp = rospy.Time.now()
		self.trajectory_publisher.publish(self.pathMsg)

		pos = np.array([trans_mat[px], trans_mat[py], trans_mat[pz]])
		min_idx = self.nearest_trajectory_point_idx(pos)
		self.visualise_nearest_trajectory_point(min_idx, trans_mat)

	def nearest_trajectory_point_idx(self, pos: np.array) -> int:
		# pos is the current [x,y,z] position of the Franka EFF
		delta = self.trajectory - pos
		dist = np.einsum('ij,ij->i', delta, delta)
		return np.argmin(dist)

	def visualise_nearest_trajectory_point(self, npIDX: int, trans_mat):
	
		br = tf2_ros.TransformBroadcaster()
		tf = transformation_matrix_to_TransformStamped(trans_mat, "panda_link0", "pNearest")

		# Change translation to the nearest position on the trajectory
		x, y, z = self.trajectory[npIDX, : ]
		nearestPos = Vector3(x,y,z)
		tf.transform.translation = nearestPos
		br.sendTransform(tf)

	def get_stiffness(self, d: np.double):
		if d < self.tunnelRadius:
			pass
		elif d < 2 * self.tunnelRadius:
			# In 
			pass
		else:
			# In fault zone
		pass

if __name__ == '__main__':
	rospy.init_node('assistance_tunnel', anonymous=True, log_level=rospy.DEBUG)
	obc = AssistiveTunnelController(tunnelRadius=0.07)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")