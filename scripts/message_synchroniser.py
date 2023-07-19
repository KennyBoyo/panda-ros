#!/usr/bin/env python3
import rospy
import sys
import message_filters
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from copy import deepcopy

"""
Class that synchronises the FrankaState and JointState messages from the Franka Panda and OpenPose
"""
class MessageSynchroniserNode:
	def __init__(self):
		"""
		Initialise message synchronise node
		"""
		self.synced_msgs = rospy.Publisher("/os3/synchronised_step_problem", JointState, queue_size=5)

		arm_state_topic = "/problem_topic"
		franka_state_topic = "/franka_state_controller/franka_states"

		arm_state_sub = message_filters.Subscriber(arm_state_topic, JointState)
		franka_state_sub = message_filters.Subscriber(franka_state_topic, FrankaState)

		ts = message_filters.ApproximateTimeSynchronizer([arm_state_sub, franka_state_sub], 10, 0.1)
		ts.registerCallback(self.callback)
		print("registered synchroniser callback")

	def callback(self, arm_state: JointState, franka_state: FrankaState):
		# The callback processing the pairs of numbers that arrived at approximately the same time
		sync: JointState = deepcopy(arm_state)
		sync.header.stamp = franka_state.header.stamp
		sync.velocity = franka_state.O_T_EE[12:15]
		sync.effort = franka_state.O_F_ext_hat_K
		self.synced_msgs.publish(sync)

def main(args):
	rospy.init_node('message_sync', anonymous=True, log_level=rospy.DEBUG)
	msn = MessageSynchroniserNode()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)