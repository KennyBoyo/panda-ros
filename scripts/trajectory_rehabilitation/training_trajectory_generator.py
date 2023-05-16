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

PARENT_FRAME = 'panda_link0'


class TrainingTrajectoryHandler():
	def __init__(self, state_topic, d_min):
		self.sub = rospy.Subscriber(state_topic,FrankaState, self.franka_callback)
		self.dist_min = d_min
		self.trajectory_publisher = rospy.Publisher("/unity/pathme", Path, queue_size=100)


		self.pbServices = [
            rospy.Service('trajectory_generation/start', Trigger, self.pb_start_handler),
            rospy.Service('trajectory_generation/stop', Trigger, self.pb_stop_handler),
            rospy.Service('trajectory_generation/add_spline', Trigger, lambda req :rospy.logwarn("Not implemented :)")),
            rospy.Service('trajectory_generation/confirm', Trigger, self.pb_confirm_handler),
        ]

		self.trajectories = [] 
		self.isRecording = False
		self.confirmFlag = False
		self.pbStates = {
			'record' : False,
			'spline' : False,
			'confirm': False
		}

		# Create nav_msgs/Path default msg
		self.pathMsg = Path()
		self.pathMsg.header.frame_id = PARENT_FRAME
		self.prev_pos = np.zeros((3,1))

	def pb_start_handler(self, req):
		self.pbStates['record'] = True
		return TriggerResponse(success=True)
	def pb_stop_handler(self, req):
		self.pbStates['record'] = False
		return TriggerResponse(success=True)
	def pb_confirm_handler(self, req):
		self.pbStates['confirm'] = True
		return TriggerResponse(success=True)

	def franka_callback(self, msg: FrankaState):

		if self.pbStates['confirm']:

			if self.isRecording:
				rospy.logwarn("Stop Recording before confirming trajectories")

			else:		
				if not self.confirmFlag:
					rospy.loginfo(f"There are {len(self.trajectories)} recorded trajectories. Press again to end session.")
					self.confirmFlag = True
					self.pbStates['confirm'] = False
				else:
					self.launch_server()
					

		if self.pbStates['record']:
			self.confirmFlag = False
			T_current = msg.O_T_EE
			p_current = np.array([T_current[12], T_current[13], T_current[14]])
			currentPose = se3_to_PoseStamped(T_current, PARENT_FRAME) 

			# First frame in path
			if self.isRecording == False:
				rospy.loginfo(f"Trajectory {len(self.trajectories)} recording begun.")
				self.pathMsg.poses.clear()
				self.pathMsg.poses.append(currentPose)
				self.isRecording = True

			elif np.linalg.norm(p_current - self.prev_pos) > self.dist_min: 
				if len(self.pathMsg.poses)%10 == 0:
					rospy.loginfo(f"Still recording Trajectory {len(self.trajectories)}")
				self.pathMsg.poses.append(currentPose)
				self.prev_pos = p_current



		if self.isRecording == True and self.pbStates['record'] == False:
			rospy.loginfo(f"Stopping trajectory recording {len(self.trajectories)}")
			print(len(self.pathMsg.poses))
			self.isRecording = False
			self.pathMsg.header.stamp = rospy.Time.now()
			self.publish_trajectory(self.pathMsg)
			self.trajectories.append(deepcopy(self.pathMsg))
	
	def publish_trajectory(self, trajectory):
		self.trajectory_publisher.publish(trajectory)
	
	def launch_server(self):
		# Disable current subscribers etc

		rospy.logwarn(f"Closing handler with {len(self.trajectories)}.")
		self.sub.unregister()
		for service in self.pbServices:
			service.shutdown("Closing service")

		# Launch server node with trajectories
		self.trajectory_server = TrajectoryServer(self.trajectories, PARENT_FRAME)


class TrajectoryServer():
	def __init__(self, trajectories: list, frame_id: str = PARENT_FRAME):
		self.trajectories = trajectories
		self.frame_id = frame_id
		self.server = rospy.Service("training_trajectory_server", TrainingTrajectory, self.get_trajectory)
		rospy.on_shutdown(self.shutdown_callback)

	def get_trajectory(self, req: TrainingTrajectoryRequest):
		
		response = TrainingTrajectoryResponse()
		response.trajectoryAvailable = False

		if req.base_frame == self.frame_id:
			if len(self.trajectories):
				response.trajectoryAvailable = True
				response.trajectory = self.trajectories[0]
				print(type(self.trajectories[0]))
				rospy.loginfo("Trajectory response with valid trajectory sent.")

		return response
	def shutdown_callback(self):
		self.server.shutdown()
		 
# def trajectory_generator(p0, n):
# 	traj_position = np.zeros((n, 3))
# 	traj_position[:, 2] = p0[2]

# 	# n_step = np.linspace(0, 0.1*np.pi, n)
# 	# traj_position[:, 0] = x0 + n_step
# 	# traj_position[:, 1] = 0.1*np.sin(10*n_step) + y0
# 	traj_position[:, 0] = p0[0]
# 	traj_position[:, 1] = p0[1] + np.linspace(0, 0.5, n)

# 	return traj_position

# def spline_trajectory_generator(p0, degree, n):
# 	cv = [[0,0,0],
# 	   		[0.1, 0.1, 0.1],
# 			[0.2, 0.2, 0.3],
# 			[0.5, -0.2, 0.2],
# 			[0.7, 0.2, -0.1]]
# 	cv = np.asarray(cv)
# 	cv += p0
# 	count = cv.shape[0]

# 	degree = np.clip(degree, 1, count - 1)
# 	kv = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

# 	# Return samples
# 	max_param = count - degree
# 	spl = scipolate.BSpline(kv, cv, degree)

# 	spline_trajectory = spl(np.linspace(0,max_param,n))
	
# 	return spline_trajectory



if __name__ == "__main__":
	rospy.init_node("trajectory_handler")
	handler = TrainingTrajectoryHandler("/franka_state_controller/franka_states", 0.05)
	rospy.spin()