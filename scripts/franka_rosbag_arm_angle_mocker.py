#!/usr/bin/env python3
import rospy
from franka_msgs.msg import FrankaState
import threading

threading.Thread



topic_name = "/franka_state_controller/franka_states"
system_info = rospy.wait_for_message(topic_name, FrankaState, timeout=1)
print(system_info)
