#!/usr/bin/env python3

from copy import deepcopy

import numpy as np
import rospy
import scipy.interpolate as scipolate
from franka_msgs.msg import FrankaState
from msg_utils import *
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse

from panda_ros.srv import *
from collections import deque


PARENT_FRAME = 'panda_link0'
p0 = np.asarray([ 0.4, -0.2,  0.3])
spline_default = [[0.3, 0, 0.3], [0.35, -0.1, 0.7], [0.6, 0.7, 0.25]]

class TrainingTrajectoryHandler():
	def __init__(self, state_topic, d_min):
		self.sub = rospy.Subscriber(state_topic,FrankaState, self.franka_callback)
		self.dist_min = d_min
		self.trajectory_publisher = rospy.Publisher("/trajectory_generation/trajectory", Path, queue_size=100)


		self.pbServices = [
            rospy.Service('trajectory_generation/start', Trigger, self.pb_start_handler),
            rospy.Service('trajectory_generation/stop', Trigger, self.pb_stop_handler),
            rospy.Service('trajectory_generation/add_default_path', Trigger, self.pb_add_default_handler),
            rospy.Service('trajectory_generation/confirm', Trigger, self.pb_confirm_handler),
        ]

		self.trajectories = deque()
		self.isRecording = False
		self.confirmFlag = False
		self.pbStates = {
			'record' : False,
			'spline' : False,
			'add_default' : False, 
			'confirm': False
		}

		# Create nav_msgs/Path default msg
		self.pathMsg = Path()
		self.pathMsg.header.frame_id = PARENT_FRAME
		self.prev_pos = np.zeros((3,1))
		
		# Launch service with trajectories
		self.trajectory_server = TrajectoryServer(self.trajectories, PARENT_FRAME)

	def pb_start_handler(self, req):
		self.pbStates['record'] = True
		return TriggerResponse(success=True)
	def pb_stop_handler(self, req):
		self.pbStates['record'] = False
		return TriggerResponse(success=True)
	def pb_confirm_handler(self, req):
		self.pbStates['confirm'] = True
		return TriggerResponse(success=True)
	def pb_add_default_handler(self, req):
		self.pbStates['add_default'] = True
		return TriggerResponse(success=True)

	def franka_callback(self, msg: FrankaState):
		T_current = msg.O_T_EE
		p_current = np.array([T_current[12], T_current[13], T_current[14]])

		if self.pbStates['confirm']:

			if self.isRecording:
				rospy.logwarn("Stop Recording before confirming trajectories")
			elif len(self.trajectories) == 0:
				rospy.logwarn("No trajectories planned. :(")
			else:		
				if not self.confirmFlag:
					rospy.loginfo(f"There are {len(self.trajectories)} recorded trajectories. Press again to end session.")
					self.confirmFlag = True
					self.pbStates['confirm'] = False
				else:
					self.confirm_server()
					
		if self.pbStates['record']:
			self.confirmFlag = False
			currentPose = se3_to_PoseStamped(T_current, PARENT_FRAME) 

			# First frame in path
			if self.isRecording == False:
				rospy.loginfo(f"Starting Trajectory no.{len(self.trajectories)} recording.")
				self.pathMsg.poses.clear()
				self.pathMsg.poses.append(currentPose)
				self.isRecording = True

			elif np.linalg.norm(p_current - self.prev_pos) > self.dist_min: 
				path_len = len(self.pathMsg.poses)
				if path_len%10 == 0:
					rospy.loginfo(f"Recording Trajectory {len(self.trajectories)}: {path_len} points.")
				self.pathMsg.poses.append(currentPose)
				self.prev_pos = p_current

		elif self.pbStates['add_default']:
			# def_traj = parameterised_trajectory(p0, 30)
			def_traj = spline_trajectory_generator(spline_default, 7, 30)
			self.publish_trajectory(def_traj)
			self.trajectories.append(def_traj)

			rospy.loginfo(f"Added default trajectory as no{len(self.trajectories)}.")
			self.pbStates['add_default'] = False

		if self.isRecording == True and self.pbStates['record'] == False:
			rospy.loginfo(f"Stopped trajectory recording no.{len(self.trajectories)}.")
			self.isRecording = False
			self.pathMsg.header.stamp = rospy.Time.now()
			self.publish_trajectory(self.pathMsg)
			self.trajectories.append(deepcopy(self.pathMsg))
	
	def publish_trajectory(self, trajectory):
		self.trajectory_publisher.publish(trajectory)
	
	def confirm_server(self):
		# Disable current subscribers etc

		rospy.logwarn(f"Closing handler with {len(self.trajectories)} trajectory(ies).")
		self.sub.unregister()
		for service in self.pbServices:
			service.shutdown("Closing pb services")

		self.trajectory_server.lock_trajectories()


class TrajectoryServer():
	def __init__(self, trajectories: deque, frame_id: str = PARENT_FRAME):
		self.trajectories = trajectories
		self.frame_id = frame_id
		self.confirm = False
		self.server = rospy.Service("training_trajectory_server", TrainingTrajectory, self.get_trajectory)
		rospy.on_shutdown(self.shutdown_callback)

	def get_trajectory(self, req: TrainingTrajectoryRequest):
		
		response = TrainingTrajectoryResponse()
		response.trajectoryAvailable = False
		
		num = len(self.trajectories)
		if req.base_frame == self.frame_id:
			if self.confirm and num > 0:
				response.trajectoryAvailable = True
				response.trajectory = self.trajectories.popleft()
				rospy.loginfo(f"Trajectory sent. {num-1} left.")

		return response
	
	def lock_trajectories(self):
		self.confirm = True
	def shutdown_callback(self):
		self.server.shutdown()
		 
def parameterised_trajectory(p0, n):
	traj_position = np.zeros((n, 3))
	traj_position[:, 2] = p0[2]

	# n_step = np.linspace(0, 0.1*np.pi, n)
	# traj_position[:, 0] = x0 + n_step
	# traj_position[:, 1] = 0.1*np.sin(10*n_step) + y0
	traj_position[:, 0] = p0[0]
	traj_position[:, 1] = p0[1] + np.linspace(0, 0.5, n)

	pathMsg = trajectory_to_Path(traj_position)
	return pathMsg

def trajectory_to_Path(trajectory):

	# Create nav_msgs/Path
	t = rospy.Time.now()
	pathMsg = Path()
	pathMsg.header.stamp = t
	pathMsg.header.frame_id = PARENT_FRAME

	poseMsg = PoseStamped()
	poseMsg.header.stamp = t
	poseMsg.header.frame_id = PARENT_FRAME

	for coord in trajectory:
		new_pose = deepcopy(poseMsg)
		new_pose.pose.position.x = coord[0]
		new_pose.pose.position.y = coord[1]
		new_pose.pose.position.z = coord[2]
		pathMsg.poses.append(new_pose)
	
	return pathMsg

def spline_trajectory_generator(cv, degree, n):
	cv = np.asarray(cv)
	count = cv.shape[0]

	degree = np.clip(degree, 1, count - 1)
	kv = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

	# Return samples
	max_param = count - degree
	spl = scipolate.BSpline(kv, cv, degree)

	spline_trajectory = spl(np.linspace(0,max_param,n))
	
	pathMsg = trajectory_to_Path(spline_trajectory)
	
	return pathMsg


if __name__ == "__main__":
	rospy.init_node("trajectory_handler")
	handler = TrainingTrajectoryHandler("/franka_state_controller/franka_states", 0.05)
	rospy.spin()