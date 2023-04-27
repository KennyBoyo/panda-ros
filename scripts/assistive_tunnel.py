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
import scipy.interpolate as scipolate

PARENT_FRAME = "panda_link0"
# X, Y and Z elements in Float64 Tmatrix (FrankaState)
px = 12
py = 13
pz = 14

K_default = np.diag([1,1,1,0,0,0])
K_default = np.array([1,1,1])

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

def trajectory_generator(p0, n):
	traj_position = np.zeros((n, 3))
	traj_position[:, 2] = p0[2]

	# n_step = np.linspace(0, 0.1*np.pi, n)
	# traj_position[:, 0] = x0 + n_step
	# traj_position[:, 1] = 0.1*np.sin(10*n_step) + y0
	traj_position[:, 0] = p0[0]
	traj_position[:, 1] = p0[1] + np.linspace(0, 0.5, n)

	return traj_position

def spline_trajectory_generator(p0, degree, n):
	cv = [[0,0,0],
       		[0.1, 0.1, 0.1],
			[0.2, 0.2, 0.4],
			[0.5, -0.2, 0.2],
			[0.7, 0.2, -0.1]]
	cv = np.asarray(cv)
	cv += p0
	count = cv.shape[0]

	degree = np.clip(degree, 1, count - 1)
	kv = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

	# Return samples
	max_param = count - degree
	spl = scipolate.BSpline(kv, cv, degree)

	spline_trajectory = spl(np.linspace(0,max_param,n))
	
	return spline_trajectory
class AssistiveTunnelController():
	def __init__(self, tunnelRadius: np.double, k_min, k_max):
		self.tunnelRadius = tunnelRadius
		self.initialisedTrajectory = False 

		self.end_effector_position_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.step )
		
		self.equilibrium_position_publisher = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped)
		self.stiffness_matrix_publisher = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_stiffness", ImpedanceParams)
		self.trajectory_publisher = rospy.Publisher("/assistance_tunnel/desired_trajectory", Path, queue_size=10)

		self.k_min = k_min
		self.k_max = k_max
		self.f_fault_tolerance_stiffness = lambda d : k_min + ((k_max - k_min)/tunnelRadius)*(d-tunnelRadius)


	def initialise_trajectory(self, t0: list):

		if not self.initialisedTrajectory:
			trans_mat = t0

			originalPose = transformation_matrix_to_PoseStamped(trans_mat, PARENT_FRAME)

			# Initialise Trajectory
			p0 = np.asarray([trans_mat[px], trans_mat[py], trans_mat[pz]])
			# self.trajectory = trajectory_generator(p0 ,n = 30)
			self.trajectory = spline_trajectory_generator(p0, 5, 50)
			# Create nav_msgs Path
			self.pathMsg = Path()
			self.pathMsg.header.stamp = rospy.Time.now()
			self.pathMsg.header.frame_id = PARENT_FRAME

			for coord in self.trajectory:
				new_pose = deepcopy(originalPose)
				new_pose.pose.position.x = coord[0]
				new_pose.pose.position.y = coord[1]
				new_pose.pose.position.z = coord[2]
				new_pose.header.stamp = rospy.Time.now()
				self.pathMsg.poses.append(new_pose)

			self.pn_idx = 0
	
	def step(self, msg: FrankaState):

		T_current = msg.O_T_EE

		# Generate trajectory (only run once)
		if not self.initialisedTrajectory:
			self.initialise_trajectory(T_current)
			self.initialisedTrajectory = True

		p_current = np.array([T_current[px], T_current[py], T_current[pz]])
		min_idx, p_min, d_min = self.nearest_point_on_trajectory(p_current, 5)

		K, p_reference = self.get_distance_model_update_parameters(d_min, p_min, p_current)
		
		# Publish updated parameters
		self.publish_updated_parameters(T_current, K, p_reference)

		# Publish trajectory and nearest point for visualisation
		self.pathMsg.header.stamp = rospy.Time.now()
		self.trajectory_publisher.publish(self.pathMsg)
		self.visualise_nearest_trajectory_point(p_reference, T_current)

	def nearest_point_on_trajectory(self, pos: np.array, vicinity_idx: int):
		prev_idx = self.pn_idx 
		
		# pos is the current [x,y,z] position of the Franka EFF
		idx_l = (prev_idx - vicinity_idx) if (prev_idx - vicinity_idx) > 0 else 0 
		traj_in_vicinity = self.trajectory[idx_l: prev_idx + vicinity_idx]
		delta = traj_in_vicinity - pos
		dist = np.sqrt(np.einsum('ij,ij->i', delta, delta))
		
		min_idx_vicinity = np.argmin(dist)
		d_min = dist[min_idx_vicinity]
		p_min = traj_in_vicinity[min_idx_vicinity]

		min_idx = idx_l + min_idx_vicinity
		self.pn_idx = min_idx
		return min_idx, p_min, d_min

	def visualise_nearest_trajectory_point(self, p_ref, trans_mat):
	
		br = tf2_ros.TransformBroadcaster()
		tf = transformation_matrix_to_TransformStamped(trans_mat, PARENT_FRAME, "p_ref")

		# Change translation to the nearest reference position on the trajectory
		x, y, z = p_ref
		referencePosition = Vector3(x,y,z)
		tf.transform.translation = referencePosition
		br.sendTransform(tf)

	def get_tolerance_tunnel_model_update_paramaters(self, d: np.double, p_min, p_current):

		# Equal stiffness of magnitude k is applied to the x, y and z axes.  
		if d < self.tunnelRadius:
			# Within inner constant force tunnel.
			k = self.k_min
			p_eqm = p_current
			rospy.loginfo(f'{"Movement Free Zone": ^30} {d:.5f}m')

		elif d < 2 * self.tunnelRadius:
			# Inside fault tolerant region. Use fault tunnel stiffness.
			k = self.f_fault_tolerance_stiffness(d)
			p_eqm = p_min
			rospy.loginfo(f"{'Fault Tolerance': ^30} {d:.5f}m")

		else:
			# In fault zone.
			k = self.k_max
			p_eqm = p_min 
			rospy.logwarn(f"{'Fault Zone': ^30} {d:.5f}m")

		# Stiffness matrix K, is 6x6
		K_stiffness = K_default * k

		return K_stiffness, p_eqm
	

	def get_distance_model_update_parameters(self, d: np.double, p_min, p_current):
		
		# Overall assistance 
		return K_default * self.k_max, p_min

	def publish_updated_parameters(self, trans_mat, K_new, p_eqm_new):
		# Equilibrium position
		eqm_msg = transformation_matrix_to_PoseStamped(trans_mat, PARENT_FRAME)
		eqm_msg.pose.position.x = p_eqm_new[0]
		eqm_msg.pose.position.y = p_eqm_new[1]
		eqm_msg.pose.position.z = p_eqm_new[2]

		self.equilibrium_position_publisher.publish(eqm_msg)
		
		# Stiffness matrix
		impedance_msg = ImpedanceParams()
		translational_stiffness_labels = ["translational_stiffness_x", "translational_stiffness_y", "translational_stiffness_z"]
		for label, value in zip(translational_stiffness_labels, K_new):
			impedance_msg.data.append(DoubleParameter(name = label, value = value))

		# translational_stiffness = DoubleParameter(name="translational_stiffness", value= list(K_new))
		impedance_msg.data.append(DoubleParameter(name="rotational_stiffness", value=0))
		impedance_msg.data.append(DoubleParameter(name="nullspace_stiffness", value=0))
		
		self.stiffness_matrix_publisher.publish(impedance_msg)

if __name__ == '__main__':
	rospy.init_node('assistance_tunnel', anonymous=True, log_level=rospy.DEBUG)
	obc = AssistiveTunnelController(tunnelRadius=0.05, k_min=0, k_max = 200)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")