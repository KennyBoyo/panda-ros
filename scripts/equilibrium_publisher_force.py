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
		self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controller/stiffness_config", StiffnessConfig
		, queue_size=5)
		self.imp = rospy.Subscriber("/cartesian_impedance_equilibrium_controller/impedance_mode", Int8, self.impedance_mode_callback)

		self.force_stiff = rospy.Publisher("/cartesian_impedance_equilibrium_controller/stiffness_config", StiffnessConfig, queue_size=5)
		self.pub_visualiser = rospy.Publisher("/cartesian_impedance_equilibrium_controller/force_direction", MarkerArray, queue_size=5)


		# self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controllerdynamic_reconfigure_compliance_param_node/parameter_updates", Config, queue_size=10)
		self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
		self.robot_pose_eq = PoseStamped()
		self.robot_pose = PoseStamped()
		self.index = 0
		self.buffer_size = 20
		self.k = 0
		self.k_t = [0] * 3


		self.mag_thres_inc = 3
		self.mag_thres_dec = 3

		self.translation_lower_limit = 0
		self.translation_upper_limit = 150
		self.robot_pose_list = [None] * self.buffer_size

		self.robot_pose.header.frame_id = "panda_link0"

		self.mode = 0

		self.wrench = WrenchStamped()
		self.force_buffer_size = 10
		self.force_buffer = [np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)] * self.force_buffer_size
		self.force_buffer_index = 0
		self.summed_force = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)



	def equilibrium_adjuster_callback(self, actual: FrankaState):

		self.generate_marker(actual)
		self.robot_pose_list[self.index] = actual
		self.index += 1
		if (self.index == self.buffer_size):
			self.index = 0
		if self.robot_pose_list[self.index] != None:
			actual = self.robot_pose_list[self.index]
			self.robot_pose.header.stamp = rospy.Time.now()

			# Convert robot end effector coordinates to quaternion
			quat = tr.quaternion_from_matrix([[actual.O_T_EE[0], actual.O_T_EE[4], actual.O_T_EE[8], actual.O_T_EE[12]], \
				[actual.O_T_EE[1], actual.O_T_EE[5], actual.O_T_EE[9], actual.O_T_EE[13]], \
					[actual.O_T_EE[2], actual.O_T_EE[6], actual.O_T_EE[10], actual.O_T_EE[14]], \
						[actual.O_T_EE[3], actual.O_T_EE[7], actual.O_T_EE[11], actual.O_T_EE[15]]])
			# # print(actual.dtheta)
			# print([[actual.O_T_EE[0], actual.O_T_EE[4], actual.O_T_EE[8], actual.O_T_EE[12]], \
			# 	[actual.O_T_EE[1], actual.O_T_EE[5], actual.O_T_EE[9], actual.O_T_EE[13]], \
			# 		[actual.O_T_EE[2], actual.O_T_EE[6], actual.O_T_EE[10], actual.O_T_EE[14]], \
			# 			[actual.O_T_EE[3], actual.O_T_EE[7], actual.O_T_EE[11], actual.O_T_EE[15]]])

			# Set end effector robot position
			self.robot_pose.pose.position.x = max([min([actual.O_T_EE[12],
																						self.position_limits[0][1]]),
																						self.position_limits[0][0]])

			self.robot_pose.pose.position.y = max([min([actual.O_T_EE[13],
																				self.position_limits[1][1]]),
																				self.position_limits[1][0]])

			self.robot_pose.pose.position.z = max([min([actual.O_T_EE[14],
																				self.position_limits[2][1]]),
																				self.position_limits[2][0]])

			# Set end effector orientation
			self.robot_pose.pose.orientation.x = quat[0]
			self.robot_pose.pose.orientation.y = quat[1]
			self.robot_pose.pose.orientation.z = quat[2]
			self.robot_pose.pose.orientation.w = quat[3]

			self.pub.publish(self.robot_pose)
			# print(self.robot_pose.pose.orientation)

			# Publish Stiffnesses
			# self.set_k(actual=actual)
			if self.mode == 1:
				self.set_k(actual)
			elif self.mode == 2:
				self.set_force_k(actual)

			# Publish pose


	# def set_k(self, actual):
	# 	stiffness_config = ImpedanceParams()
	# 	stiffness_config.headers.stamp = rospy.Time.now()
	# 	index_delay = 1
	# 	magnitude = (20*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2 + (actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2 + (actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5)
	# 	magnitudes = ((30*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2)**0.5), (30*((actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2)**0.5), (30*((actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5))

	# 	#SIGMOID
	# 	for i in range(len(magnitudes)):
	# 		if (magnitudes[i] < self.mag_thres_dec):
	# 			self.k_t[i] -= 1
	# 		elif (magnitudes[i] > self.mag_thres_inc):
	# 			self.k_t[i] += magnitudes[i]

	# 		if (self.k_t[i] < 0):
	# 				self.k_t[i] = 0

	# 	if (magnitude < self.mag_thres_dec):
	# 			self.k -= 1
	# 	elif (magnitude > self.mag_thres_inc):
	# 		self.k += magnitude

	# 	if (self.k < 0):
	# 			self.k = 0

	# 	translation_stiffness_labels = ["translational_stiffness_x", "translational_stiffness_y", "translational_stiffness_z"]

	# 	if (self.mode == 0):
	# 			stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=0))
	# 			rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=0)
	# 			nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=0)

	# 	else:
	# 		for i in range(len(magnitudes)):
	# 			if (self.k_t[i] < self.translation_lower_limit):
	# 				stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=0))
	# 			elif (self.k_t[i] > self.translation_upper_limit):
	# 				self.k_t[i] = self.translation_upper_limit
	# 				stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=self.translation_upper_limit))
	# 			else:
	# 				stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=self.k_t[i]))


	# 		if (self.k < self.translation_lower_limit):
	# 			rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=0)
	# 			nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=0)
	# 		elif (self.k > self.translation_upper_limit):
	# 			self.k = self.translation_upper_limit
	# 			rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.translation_upper_limit/3)
	# 			nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)
	# 		else:
	# 			rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.k/3)
	# 			nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)

	# 	stiffness_config.data.append(rotational_stiffness)
	# 	stiffness_config.data.append(nullspace_stiffness)

	# 	stiffness_config.data.append(DoubleParameter(name="mode", value=self.mode))

	# 	self.stiffness.publish(stiffness_config)

	def set_k(self, actual):
		stiffness_config: StiffnessConfig = StiffnessConfig()
		stiffness_config.headers.stamp = rospy.Time.now()
		index_delay = 1
		magnitude = (20*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2 + (actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2 + (actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5)
		magnitudes = ((30*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2)**0.5), (30*((actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2)**0.5), (30*((actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5))

		#SIGMOID
		for i in range(len(magnitudes)):
			if (magnitudes[i] < self.mag_thres_dec):
				self.k_t[i] -= 1
			elif (magnitudes[i] > self.mag_thres_inc):
				self.k_t[i] += magnitudes[i]

			if (self.k_t[i] < 0):
					self.k_t[i] = 0

		if (magnitude < self.mag_thres_dec):
				self.k -= 1
		elif (magnitude > self.mag_thres_inc):
			self.k += magnitude

		if (self.k < 0):
				self.k = 0


		# Assign Translational Stiffnesses
		translation_stiffness = np.zeros((3, 3))
		if (self.mode == 1):
			for i in range(len(magnitudes)):
				if (self.k_t[i] < self.translation_lower_limit):
					translation_stiffness[i, i] = 0
				elif (self.k_t[i] > self.translation_upper_limit):
					self.k_t[i] = self.translation_upper_limit
					translation_stiffness[i, i] = self.translation_upper_limit
				else:
					translation_stiffness[i, i] = self.k_t[i]

		stiffness_config.force = translation_stiffness.reshape(-1)
		stiffness_config.torque = 30 * np.eye(3).reshape(-1)
		self.stiffness.publish(stiffness_config)

	
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
		f_mat = np.abs(self.get_rotated_ellipsoid(f_mag, f_vec, normal_scale).reshape(-1))
		t_mat = np.abs(self.get_rotated_ellipsoid(t_mag, t_vec, normal_scale).reshape(-1))

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
		
		stiffness_config.torque = 30 * np.eye(3).reshape(-1)

		self.force_stiff.publish(stiffness_config)

	def get_rotated_ellipsoid(self, mag, dir_vec, normal_scale):
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
		f_mat = np.abs(self.get_rotated_ellipsoid(f_mag, f_vec, 1))
		t_mat = np.abs(self.get_rotated_ellipsoid(t_mag, t_vec, 1))
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

		
def main(args):
	rospy.init_node('equilibrium_publisher', anonymous=True, log_level=rospy.DEBUG)
	obc = equilibrium_publisher()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)