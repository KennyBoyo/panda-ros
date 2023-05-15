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

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    if (s == 0):
        return np.eye(3)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

class equilibrium_publisher:

	def __init__(self):
		self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.equilibrium_adjuster_callback)
		self.pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped, queue_size=5)
		self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_stiffness", ImpedanceParams, queue_size=5)
		self.imp = rospy.Subscriber("/cartesian_impedance_equilibrium_controller/impedance_mode", Int8, self.impedance_mode_callback)

		self.force_stiff = rospy.Publisher("/cartesian_impedance_equilibrium_controller/stiffness_config", StiffnessConfig, queue_size=5)
		self.pub_visualiser = rospy.Publisher("/cartesian_impedance_equilibrium_controller/force_direction", MarkerArray, queue_size=5)


		# self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controllerdynamic_reconfigure_compliance_param_node/parameter_updates", Config, queue_size=10)
		# self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
		self.robot_pose_eq = PoseStamped()
		self.robot_pose = PoseStamped()
		self.pose_index = 0
		self.pose_index_delay = 19
		self.pose_buffer_size = 20
		self.k = 0
		self.k_t = [0] * 3


		self.mag_thres_inc = 3
		self.mag_thres_dec = 3

		self.translation_lower_limit = 0
		self.translation_upper_limit = 150
		self.robot_pose_list = [None] * self.pose_buffer_size

		self.robot_pose.header.frame_id = "panda_link0"

		self.mode = 0

		# Force logging
		self.force_buffer_size = 10
		self.force_buffer = [np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)] * self.force_buffer_size
		self.force_buffer_index = 0
		self.summed_force = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

		# This is the initial stiffness matrix. Be carful with changing this, 
		# since a non-positive definite matrix will cause instability and the robot will break.
		self.stiff_init_max = 200
		self.stiff_init_min = 50
		self.max = np.array([self.stiff_init_max, self.stiff_init_min, self.stiff_init_min, 
		       self.stiff_init_min, self.stiff_init_max, self.stiff_init_min, 
			   self.stiff_init_min, self.stiff_init_min, self.stiff_init_max], dtype=np.float64)
		self.current_stiffness = deepcopy(self.max)

		
		self.v_thres_low = 0.1
		self.v_thres_high = 0.15
		self.f_thres_low = 4
		self.adjustment_queue = []


	def equilibrium_adjuster_callback(self, state: FrankaState):
		self.robot_pose_list[self.pose_index] = state
		self.pose_index += 1
		if (self.pose_index == self.pose_buffer_size):
			self.pose_index = 0
		if self.robot_pose_list[self.pose_index] != None:

			equilibrium_pose = self.robot_pose_list[(self.pose_index - self.pose_index_delay) % 20]
			self.robot_pose.header.stamp = rospy.Time.now()

			# Convert robot end effector coordinates to quaternion
			quat = tr.quaternion_from_matrix([[equilibrium_pose.O_T_EE[0], equilibrium_pose.O_T_EE[4], equilibrium_pose.O_T_EE[8], equilibrium_pose.O_T_EE[12]], \
											[equilibrium_pose.O_T_EE[1], equilibrium_pose.O_T_EE[5], equilibrium_pose.O_T_EE[9], equilibrium_pose.O_T_EE[13]], \
											[equilibrium_pose.O_T_EE[2], equilibrium_pose.O_T_EE[6], equilibrium_pose.O_T_EE[10], equilibrium_pose.O_T_EE[14]], \
											[equilibrium_pose.O_T_EE[3], equilibrium_pose.O_T_EE[7], equilibrium_pose.O_T_EE[11], equilibrium_pose.O_T_EE[15]]])
			
			# Set end effector robot position
			self.robot_pose.pose.position.x = equilibrium_pose.O_T_EE[12]

			self.robot_pose.pose.position.y = equilibrium_pose.O_T_EE[13]

			self.robot_pose.pose.position.z = equilibrium_pose.O_T_EE[14]

			# Set end effector orientation
			self.robot_pose.pose.orientation.x = quat[0]
			self.robot_pose.pose.orientation.y = quat[1]
			self.robot_pose.pose.orientation.z = quat[2]
			self.robot_pose.pose.orientation.w = quat[3]
			self.generate_marker(state)

			# Publish equilibrium pose
			self.pub.publish(self.robot_pose)

			# Publish updated stiffnesses
			self.adjust_stiffness(state)
	
	def set_force_k(self, state):
		# Get Force relative to Origin frame
		wrench_o = state.O_F_ext_hat_K
		f_vec = np.array([wrench_o[0], wrench_o[1], wrench_o[2]])
		t_vec = np.array([wrench_o[3], wrench_o[4], wrench_o[5]])
		
		# Get Force and torque magnitude
		f_mag = np.linalg.norm(f_vec*20)
		t_mag = np.linalg.norm(t_vec*20)

		# If force or wrench is exactly zero, we are approaching joint limit so don't do anything
		if (f_mag == 0 or t_mag == 0):
			return
		
		# Initialise StiffnessConfig message
		stiffness_config = StiffnessConfig()
		stiffness_config.headers.stamp = rospy.Time.now()

		# Setup buffer
		self.force_buffer_index += 1
		if self.force_buffer_index == self.force_buffer_size:
			self.force_buffer_index = 0
		normal_scale = 1/5

		# Get rotated Stiffness ellipsoid in direction of force exertion 
		f_mat = np.abs(self.get_rotated_ellipsoid(f_vec, f_mag, normal_scale).reshape(-1))
		t_mat = np.abs(self.get_rotated_ellipsoid(t_vec, t_mag, normal_scale).reshape(-1))

		# Buffer new value
		self.force_buffer[self.force_buffer_index] = f_mat

		# Transition out old bufferred force values
		self.summed_force += f_mat
		self.summed_force -= self.force_buffer[(self.force_buffer_index-(self.force_buffer_size-1)) % self.force_buffer_size]
		
		# Fill out Message values
		stiffness_config.force = self.summed_force / self.force_buffer_size
		stiffness_config.torque = t_mat
		stiffness_config.force_mag = f_mag
		stiffness_config.torque_mag = t_mag

		self.force_stiff.publish(stiffness_config)

	def get_rotated_ellipsoid(self, dir_vec, mag = 1, normal_scale = 0.1):
		axis = np.array([1, 0, 0])
		rotation = rotation_matrix_from_vectors(axis, dir_vec)

		k_mat = np.zeros((3,3))
		k_mat[0][0] = mag
		k_mat[1][1] = mag * normal_scale
		k_mat[2][2] = mag * normal_scale

		k_mat = rotation @ k_mat @ rotation.T
		return k_mat

	def impedance_mode_callback(self, msg):
		rospy.loginfo(self.mode)
		self.reset_stiffness()
		self.mode = msg.data

	def generate_marker(self, state):
		wrench_markers = MarkerArray()
		force_marker = Marker()
		torque_marker = Marker()
		# Get Force relative to Origin frame
		wrench_o = state.O_F_ext_hat_K
		f_vec = np.array([wrench_o[0], wrench_o[1], wrench_o[2]])
		t_vec = np.array([wrench_o[3], wrench_o[4], wrench_o[5]])
		
		# Get Force and torque magnitude
		f_mag = np.linalg.norm(f_vec*20)
		t_mag = np.linalg.norm(t_vec*20)

		# If force or wrench is exactly zero, we are approaching joint limit so don't do anything
		if (f_mag == 0 or t_mag == 0):
			return
		
		# Get rotated Stiffness ellipsoid in direction of force exertion 
		f_mat = np.abs(self.get_rotated_ellipsoid(f_vec, f_mag, 1))
		t_mat = np.abs(self.get_rotated_ellipsoid(t_vec, t_mag, 1))
		f_mat = rotation_matrix_from_vectors(np.array([1, 0, 0]), f_vec)
		t_mat = rotation_matrix_from_vectors(np.array([1, 0, 0]), t_vec)

		T_force = [[f_mat[0][0], f_mat[0][1], f_mat[0][2], state.O_T_EE[12]], \
					[f_mat[1][0], f_mat[1][1], f_mat[1][2], state.O_T_EE[13]], \
					[f_mat[2][0], f_mat[2][1], f_mat[2][2], state.O_T_EE[14]], \
					[0, 0, 0, 1]]
		
		force_quat = tr.quaternion_from_matrix(T_force)
		force_marker.id = 23
		force_marker.header.frame_id = self.robot_pose.header.frame_id
		force_marker.pose.position.x = state.O_T_EE[12]
		force_marker.pose.position.y = state.O_T_EE[13]
		force_marker.pose.position.z = state.O_T_EE[14]
		force_quat /= np.linalg.norm(force_quat)
		force_marker.pose.orientation.x = force_quat[0]
		force_marker.pose.orientation.y = force_quat[1]
		force_marker.pose.orientation.z = force_quat[2]
		force_marker.pose.orientation.w = force_quat[3]
		force_marker.color.a = 1.0
		force_marker.color.r = 0.0
		force_marker.color.g = 1.0
		force_marker.color.b = 0.0

		force_marker.type = Marker.ARROW
		force_marker.header.stamp = rospy.Time.now()
		force_marker.scale.x = f_mag/200
		force_marker.scale.y = 0.01
		force_marker.scale.z = 0.01
		
		wrench_markers.markers.append(force_marker)
		self.pub_visualiser.publish(wrench_markers)

	def adjust_stiffness(self, state, verbose=False):
		# Initialise StiffnessConfig message
		stiffness_config = StiffnessConfig()
		stiffness_config.headers.stamp = rospy.Time.now()

		# Get Force relative to Origin frame
		wrench_o = state.O_F_ext_hat_K
		f_vec = np.array([wrench_o[0], wrench_o[1], wrench_o[2]])
		t_vec = np.array([wrench_o[3], wrench_o[4], wrench_o[5]])

		# Calculate force vector
		f_mag = np.linalg.norm(f_vec)
		try:
			# if mag is 0, skip to avoid divide by zero
			f_dir = f_vec / f_mag
		except:
			return

		# v = d / dt
		index_delay = 10
		dist = np.linalg.norm([self.robot_pose.pose.position.x - self.robot_pose_list[(self.pose_index - index_delay) % self.pose_buffer_size].O_T_EE[12],\
							self.robot_pose.pose.position.y - self.robot_pose_list[(self.pose_index - index_delay) % self.pose_buffer_size].O_T_EE[13],\
							self.robot_pose.pose.position.z - self.robot_pose_list[(self.pose_index - index_delay) % self.pose_buffer_size].O_T_EE[14]])
		
		dt = self.robot_pose.header.stamp.to_sec() - self.robot_pose_list[(self.pose_index - index_delay) % self.pose_buffer_size].header.stamp.to_sec()
		
		vel = dist/dt
		
		if (verbose):
			print("Time =", dt)
			print("Force =", f_mag)
			print("Velocity =", vel)
			print("Distance =", dist)
		
		f_mat = self.get_rotated_ellipsoid(f_dir).reshape(-1)

		# Adjustment matrix is a unit step in the direction of force
		adjustment_matrix = 2*np.array(f_mat/np.linalg.norm(f_mat), dtype = np.float64)
		# print(adjustment_matrix.shape)

		if (vel < self.v_thres_low):
			if (f_mag > self.f_thres_low):
				temp_stiff = self.current_stiffness - adjustment_matrix
				for i in range(len(temp_stiff)):
					if temp_stiff[i] < 0:
						adjustment_matrix[i] += temp_stiff[i]
						temp_stiff[i] = 0
					
				temp_stff_mat = temp_stiff.reshape(3, 3)
				det = np.linalg.det(temp_stff_mat)
				if det < 0:
					# adjustment_matrix = np.zeros(adjustment_matrix.shape[0])
					return
				
				# print(det)

				self.adjustment_queue.append(adjustment_matrix)
				self.current_stiffness -= adjustment_matrix
			else:
				if (self.adjustment_queue.__len__() > 0):
					self.current_stiffness += self.adjustment_queue.pop()


		elif (vel > self.v_thres_high):
			if (self.adjustment_queue.__len__() > 0):
				self.current_stiffness += self.adjustment_queue.pop()

		# Fill out Message values
		stiffness_config.force = self.current_stiffness
		# stiffness_config.torque = t_mat
		stiffness_config.force_mag = f_mag
		# stiffness_config.torque_mag = t_mag

		if (verbose):
			print(self.current_stiffness)

		self.force_stiff.publish(stiffness_config)

	def reset_stiffness(self):
		self.current_stiffness = deepcopy(self.max)
		self.adjustment_queue = []

		


		
def main(args):
	rospy.init_node('equilibrium_publisher', anonymous=True, log_level=rospy.DEBUG)
	obc = equilibrium_publisher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)