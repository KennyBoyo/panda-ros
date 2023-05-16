#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr
import numpy as np
from copy import deepcopy
import sys, select, termios, tty
import scipy.interpolate as scipolate

PARENT_FRAME = 'panda_link0'

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
			[0.2, 0.2, 0.3],
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
	msg.pose.position.x = trans_mat[12]
	msg.pose.position.y = trans_mat[13]
	msg.pose.position.z = trans_mat[14]

	return msg
class TrajectoryUpdater():
    def __init__(self):
        self.sub = rospy.Subscriber("/franka_state_controller/franka_states",FrankaState, self.record_position)
        self.trajectory_publisher = rospy.Publisher("/unity/path", Path, queue_size=100)

        self.trajectory = [] 
        self.initialised = False

    def record_position(self, msg: FrankaState):

        T_current = msg.O_T_EE
        p_current = np.array([T_current[12], T_current[13], T_current[14]])
        if not self.initialised:
            self.trajectory.append(p_current)
            self.originalPose = transformation_matrix_to_PoseStamped(T_current, PARENT_FRAME)
            self.initialised = True
        else:
            dist = np.linalg.norm(p_current- self.trajectory[-1])
            if dist > 0.03:
                self.trajectory.append(p_current)

        if len(self.trajectory) > 20:
            print("published")
            self.publish_trajectory()


        # if len(self.trajectory) > 50:
        #     rospy.signal_shutdown("because")


    def publish_trajectory(self):


        # Create nav_msgs Path
        self.pathMsg = Path()
        self.pathMsg.header.stamp = rospy.Time.now()
        self.pathMsg.header.frame_id = PARENT_FRAME

        for coord in self.trajectory:
            new_pose = deepcopy(self.originalPose)
            new_pose.pose.position.x = coord[0]
            new_pose.pose.position.y = coord[1]
            new_pose.pose.position.z = coord[2]
            new_pose.header.stamp = rospy.Time.now()
            self.pathMsg.poses.append(new_pose)

        self.trajectory_publisher.publish(self.pathMsg)

# class TrajectoryServer():
#      def __init__(self):
#           self.server = rospy.Servi
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN,termios.tcgetattr(sys.stdin))
    return key

if __name__ == "__main__":
    rospy.init_node("training_trajectory")
    traj = TrajectoryUpdater()
    # while not rospy.is_shutdown():

    #     key = getKey()
    #     if (key == '\x03'):
    #         break
    #     elif(key == "i"):
    #         print("hello")
    rospy.spin()