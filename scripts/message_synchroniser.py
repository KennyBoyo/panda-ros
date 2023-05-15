#!/usr/bin/env python3
import rospy
import sys
import message_filters
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from copy import deepcopy


class MessageSynchroniserNode:
	def __init__(self):
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
		# print("entering callback")
		sync: JointState = deepcopy(arm_state)
		sync.header.stamp = franka_state.header.stamp
		# sync.header.stamp = rospy.Time.now()
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