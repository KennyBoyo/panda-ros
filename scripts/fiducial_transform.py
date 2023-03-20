#!/usr/bin/env python3

import roslib
import rospy
import tf
from fiducial_msgs.msg import FiducialTransformArray

def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), 'fiducial_0', 'panda_hand')

if __name__ == '__main__':
    rospy.init_node('aruco_to_panda_hand')
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()