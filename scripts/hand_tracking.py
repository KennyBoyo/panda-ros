#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from time import sleep



class hand_tracker:

  def __init__(self):
    self.sub = rospy.Subscriber("/mrn_vision/openpose/body/wrist_right", PointStamped, self.wrist_callback)
    self.pub = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=1)
    self.mode = rospy.Subscriber("/cartesian_impedance_equilibrium_controller/mode", Int8, self.mode_callback)
    self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
    # self.position_limits = [[-1, 1], [-1, 1], [0.1, 1]]
    self.robot_pose = PoseStamped()
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    # self.robot_pose.pose.position.x = 0.2061527818441391
    # self.robot_pose.pose.position.y = 0.0015579211758449674
    # self.robot_pose.pose.position.z = 0.36875462532043457
    # self.robot_pose.pose.orientation.x = 0.997001051902771
    # self.robot_pose.pose.orientation.y = -0.03093821555376053
    # self.robot_pose.pose.orientation.z = -0.0690128430724144
    # self.robot_pose.pose.orientation.w = -0.016413375735282898
    # self.pub.publish(self.robot_pose)

    self.is_running = 0

    # sleep(10)


    
    # self.wrist_sub = message_filters.Subscriber("/mrn_vision/openpose/body/wrist_right", PointStamped)
    # self.elbow_sub = message_filters.Subscriber("/mrn_vision/openpose/body/elbow_right", PointStamped)

    # self.ts = message_filters.TimeSynchronizer([self.wrist_sub, self.elbow_sub], 10)
    # self.ts.registerCallback(self.callback)

  # def callback(self, wrist, elbow):
  #   # self.transform = self.tf_buffer.lookup_transform("frame_color_097377233947", "panda_link0", rospy.Time.now())
  #   # self.transformation = self.tf_buffer.transform(data, "panda_link0")
  #   self.wrist_pos = self.tf_buffer.transform(wrist, "panda_link0")
  #   self.elbow_pos = self.tf_buffer.transform(elbow, "panda_link0")

  #   self.vector = Point()
  #   self.vector.x = self.wrist_pos.point.x + 0.2 * (self.wrist_pos.point.x - self.elbow_pos.point.x)
  #   self.vector.y = self.wrist_pos.point.y + 0.2 * (self.wrist_pos.point.y - self.elbow_pos.point.y)
  #   self.vector.z = self.wrist_pos.point.z + 0.2 * (self.wrist_pos.point.z - self.elbow_pos.point.z)
    
  #   self.robot_pose.pose.position.x = max([min([self.vector.x,
  #                                         self.position_limits[0][1]]),
  #                                         self.position_limits[0][0]])
  #   self.robot_pose.pose.position.y = max([min([self.vector.y,
  #                                     self.position_limits[1][1]]),
  #                                     self.position_limits[1][0]])
  #   self.robot_pose.pose.position.z = max([min([self.vector.z,
  #                                     self.position_limits[2][1]]),
  #                                     self.position_limits[2][0]])
  #   self.robot_pose.pose.orientation.x = 1
  #   self.robot_pose.pose.orientation.y = 0
  #   self.robot_pose.pose.orientation.z = 0
  #   self.robot_pose.pose.orientation.w = 0
  #   self.pub.publish(self.robot_pose)

  def wrist_callback(self, wrist):
    # self.transform = self.tf_buffer.lookup_transform("frame_color_097377233947", "panda_link0", rospy.Time.now())
    self.vector = self.tf_buffer.transform(wrist, "panda_link0")
    
    self.robot_pose.pose.position.x = max([min([self.vector.point.x,
                                          self.position_limits[0][1]]),
                                          self.position_limits[0][0]])
    self.robot_pose.pose.position.y = max([min([self.vector.point.y,
                                      self.position_limits[1][1]]),
                                      self.position_limits[1][0]])
    self.robot_pose.pose.position.z = max([min([self.vector.point.z,
                                      self.position_limits[2][1]]),
                                      self.position_limits[2][0]])
    self.robot_pose.pose.orientation.x = 1
    self.robot_pose.pose.orientation.y = 0
    self.robot_pose.pose.orientation.z = 0
    self.robot_pose.pose.orientation.w = 0

    print(self.is_running)
    if (self.is_running == 1):
      print(self.is_running)
      self.pub.publish(self.robot_pose)

  def mode_callback(self, msg):
    self.is_running = msg.data


def main(args):
  rospy.init_node('hand_tracker', anonymous=True)
  obc = hand_tracker()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)