import numpy as np 
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
import tf.transformations as tr

def se3_to_PoseStamped(trans_mat, frame_id: str) -> PoseStamped:
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

def se3_to_TransformStamped(trans_mat, frame_id: str, child_id: str) -> TransformStamped:
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
	msg.transform.translation.x = trans_mat[12]
	msg.transform.translation.y = trans_mat[13]
	msg.transform.translation.z = trans_mat[14]

	return msg