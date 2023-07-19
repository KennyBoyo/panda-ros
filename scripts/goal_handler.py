#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Pose
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from franka_msgs.msg import FrankaState
from nav_msgs.msg import Path

class goal_handler:
	"""
	Class that handles goal setting and goal progression
	"""
	def __init__(self):
		"""
		Initialisation function for the goal handler, 
		"""
		self.robot_state_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.update_goal)
		self.new_goal_sub = rospy.Subscriber("/unity/trajectory", Path, self.set_new_goals)
		self.goal_pub = rospy.Publisher("/upper_limb_impedance_assessment/goal", PoseStamped, queue_size=5)
		self.goal_marker_pub = rospy.Publisher("/upper_limb_impedance_assessment/goal_marker", Marker, queue_size=5)
		self.unity_pub = rospy.Publisher("/unity/current_goal", JointState, queue_size=5)

		self.robot_pose: PoseStamped = PoseStamped()
		self.goals_list = []
		self.current_goal_index = 0
		
		self.robot_pose.header.frame_id = "panda_link0"
		self.pos_marker = Marker()
		self.goal_marker = Marker()

		# Resolution of cube (10^(-cube_res))
		self.cube_res_places = 2
		self.cube_res_factor = 5
		self.cube_res = self.cube_res_factor * 10**(-self.cube_res_places)

		# Side width of cube workspace (meters) (min is 2x max reach of robot)
		self.cube_len = 4
		self.cube_grid = np.zeros((int(self.cube_len/self.cube_res), int(self.cube_len/self.cube_res), int(self.cube_len/self.cube_res)), dtype=np.bool8)

		# Unique Marker ID
		self.idx = 0
		self.init_goal_handler()


	def init_goal_handler(self):
		"""
		Initialises the initial goal set for the system. This should be investigated and changed in the future according to 
		the future directions section of the thesis.
		"""
		self.goals_list = [self.gen_pose(0.54, 0.55, 0.48), self.gen_pose(0.38, -0.17, 0.35), self.gen_pose(0.47, 0.21, 0.31), self.gen_pose(0.26, 0, 0.83), self.gen_pose(0.29, -0.16, 0.56), self.gen_pose(0.45, 0.48, 0.35), self.gen_pose(0.45, 0.48, 0.35), self.gen_pose(0.4, 0.7, 0.25), self.gen_pose(0.5, 0.25, 0.5), self.gen_pose(0.5, -0.2, 0.5), self.gen_pose(0.5, 0.25, 0.55), self.gen_pose(0.35, -0.25, 0.33),]
		self.current_goal_index = 0
		self.sequence = np.random.permutation(len(self.goals_list))
		self.init_goal_marker()

	def set_new_goals(self, new_goals: Path):
		"""
		Receives a new set of goals from a topic sets the current goal set to be equal to the new goal set.
		"""
		self.goals_list = new_goals.poses
		self.sequence = np.random.permutation(len(self.goals_list))
		self.current_goal_index = 0

	def gen_pose(self, xp, yp, zp, xo=1, yo=0, zo=0, wo=1, scale=1):
		"""
		Convenience function to generate a PoseStamped object given a position and orientation

		Args:
			xp (int): x position
			yp (int): y position
			zp (int): z position
			xo (int, optional): x orientation. Defaults to 1.
			yo (int, optional): y orientation. Defaults to 0.
			zo (int, optional): z orientation. Defaults to 0.
			wo (int, optional): w orientation. Defaults to 1.
			scale (int, optional): scaling of the position. Defaults to 1.

		Returns:
			_type_: A default pose stamped object with set position and orientation
		"""
		pose = PoseStamped()
		pose.pose.position.x = scale * xp
		pose.pose.position.y = scale * yp
		pose.pose.position.z = scale * zp
		pose.pose.orientation.x = scale * xo
		pose.pose.orientation.y = scale * yo
		pose.pose.orientation.z = scale * zo
		pose.pose.orientation.w = scale * wo
		return pose

	def pub_current_goal(self):
		"""
		Publish the current goal
		"""
		self.goal_pub.publish(self.goals_list[self.current_goal_index])

	def update_goal(self, state: FrankaState):
		"""
		Update the current goal position to the next in the sequence

		Args:
			state (FrankaState): the current state of the panda
		"""
		self.update_goal_marker(state)

	def init_goal_marker(self):
		"""
		Initialise the goal marker object so that it does not need to be reinstantiated
		"""
		self.pos_marker.id = self.idx+1
		self.pos_marker.header.frame_id = self.robot_pose.header.frame_id
		self.pos_marker.pose.orientation.x = 0
		self.pos_marker.pose.orientation.y = 0
		self.pos_marker.pose.orientation.z = 0
		self.pos_marker.pose.orientation.w = 1
		self.pos_marker.color.a = 1
		self.pos_marker.color.r = 1.0
		self.pos_marker.color.g = 1.0
		self.pos_marker.color.b = 0.0
		self.pos_marker.type = Marker.SPHERE
		self.pos_marker.header.stamp = rospy.Time.now()
		self.pos_marker.scale.x = self.cube_res
		self.pos_marker.scale.y = self.cube_res
		self.pos_marker.scale.z = self.cube_res

	def update_goal_marker(self, state: FrankaState):
		"""
		Check if the current goal marker has been reached and update if required

		Args:
			state (FrankaState): Current state of the panda
		"""
		if len(self.goals_list) == 0:
			return

		# Snap current panda end effector(EE) and goal positions to grid for easy comparison
		x, y, z = self.snap_grid(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14])
		xg, yg, zg = self.snap_grid(self.goals_list[self.current_goal_index].pose.position.x, self.goals_list[self.current_goal_index].pose.position.y, self.goals_list[self.current_goal_index].pose.position.z)
		
		# Check if the current goal has been reached
		goal_reached = self.check_goal_reached([x, y, z], [xg, yg, zg])
		
		# Instantiate a new point to be set as the current goal
		point = Point()
		point.x = xg
		point.y = yg
		point.z = zg

		self.pos_marker.pose.position = point

		# Publish Goal Position
		self.pub_to_unity(xg, yg, zg, self.pos_marker.color.a, self.pos_marker.color.r, self.pos_marker.color.g, self.pos_marker.color.b)
		self.goal_marker_pub.publish(self.pos_marker)
		self.pub_current_goal()

	
	def check_goal_reached(self, p1, p2):
		"""
		Checks if the current goal has been reached

		Args:
			p1 (array[int]): Coordinate of first point
			p2 (array[int]): Coordinate of second point (usually set to goal)

		Returns:
			_type_: _description_
		"""
		p1 = np.array(p1)
		p2 = np.array(p2)
		if np.around(np.linalg.norm(p1 - p2), self.cube_res_places)  == 0.:
			self.current_goal_index += 1
			if self.current_goal_index == len(self.goals_list):
				self.current_goal_index = 0
				self.sequence = np.random.permutation(len(self.goals_list))
				if verbose:
					print("goal", self.current_goal_index)
					print("seq", self.sequence)
			return True
		return False
	
	def pub_to_unity(self, x, y, z, a, r, g, b):
		"""
		Publish current goal to unity

		Args:
			x (float): x position of goal
			y (float): y position of goal
			z (float): z position of goal
			a (float): Opacity of marker
			r (float): Red colour of marker
			g (float): Green colour of marker
			b (float): Blue colour of marker
		"""
		js = JointState()
		js.header.stamp = rospy.Time.now()

		js.position = [x, y, z]
		js.velocity = [r, g, b, a]
		js.effort = [self.cube_res]

		self.unity_pub.publish(js)

	def snap_grid(self, x, y, z):
		"""
		Snaps a point to the closest point on the grid defined in the init function of the class

		Args:
			x (float): x position of the point
			y (float): y position of the point
			z (float): z position of the point

		Returns:
			_type_: a point representing the closest point on the grid to the position specified
		"""
		if self.cube_res_factor > 1:
			return np.around(x*(10/self.cube_res_factor), self.cube_res_places-1)/(10/self.cube_res_factor), np.around(y*(10/self.cube_res_factor), self.cube_res_places-1)/(10/self.cube_res_factor), np.around(z*(10/self.cube_res_factor), self.cube_res_places-1)/(10/self.cube_res_factor)
		return np.around(x*(10/self.cube_res_factor), self.cube_res_places)/(10/self.cube_res_factor), np.around(y*(10/self.cube_res_factor), self.cube_res_places)/(10/self.cube_res_factor), np.around(z*(10/self.cube_res_factor), self.cube_res_places)/(10/self.cube_res_factor)
		
def main(args):
	rospy.init_node('goal_handler', anonymous=True, log_level=rospy.DEBUG)
	obc = goal_handler()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
		main(sys.argv)