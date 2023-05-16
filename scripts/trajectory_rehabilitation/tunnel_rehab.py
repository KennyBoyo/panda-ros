#!/usr/bin/env python3

import rospy
import numpy as np 
import tf2_ros
from copy import deepcopy 
from franka_msgs.msg import FrankaState
from nav_msgs.msg import Path
from panda_ros.msg import StiffnessConfig
from geometry_msgs.msg import PoseStamped, Vector3
from panda_ros.srv import *
from msg_utils import *
PARENT_FRAME = "panda_link0"
# Honing Parameters
hone_dist = 50e-3 # m
# Constants
K_default =  np.diag([1,1,1,0,0,0])
MIN_IDX = 0
MAX_IDX = 1
# States
IDLE_STATE = 0
NEW_TRAJECTORY_STATE = 1
HONING_STATE = 2
EXECUTE_STATE = 3


class TunnelTrajectoryController():
    def __init__ (self, tunnel_radius: np.double, k_min, k_max):
        self.tunnel_radius = tunnel_radius
        self.stiffness_limits = (k_min, k_max)
        self.currentState = IDLE_STATE
        self.pn_idx = 0
        self.f_fault_tolerance_stiffness = lambda d : k_min + ((k_max - k_min)/tunnel_radius)*(d-tunnel_radius)

        # Wait for trajectory server    
        rospy.wait_for_service("/training_trajectory_server")
        # Incoming:
        self.pEFF_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.step)
        self.trajectory_srv = rospy.ServiceProxy("training_trajectory_server", TrainingTrajectory)
        # Outgoing:
        self.pEqm_pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped, queue_size=10)
        self.stiffnessParams_pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/stiffness_config", StiffnessConfig, queue_size=10)
        self.trajectory_pub = rospy.Publisher("/unity/desired_trajectory", Path, queue_size=10)

    def step(self, msg: FrankaState):
        state = self.currentState
        T_current = msg.O_T_EE
        p_current = np.array([T_current[12], T_current[13], T_current[14]])

        if state == IDLE_STATE:
            try:
                req = self.trajectory_srv(PARENT_FRAME)
                req: TrainingTrajectoryResponse
                if req.trajectoryAvailable:
                    self.trajectory_path = req.trajectory
                    self.currentState = NEW_TRAJECTORY_STATE

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s"%e)

        else:

            if state != EXECUTE_STATE:

                if state == NEW_TRAJECTORY_STATE:
                    self.initialise_new_trajectory()
                    self.currentState = HONING_STATE
                    
                elif state == HONING_STATE:
                    d = np.linalg.norm(p_current - self.trajectory_positions[0])
                    rospy.logwarn_throttle(2 ,f"Please hone to the start: {d*1e3} millimeters away. Control pose of EFF.")
                    if d < hone_dist:
                        rospy.loginfo("EFF honed. Starting assistive rehabilitation.")
                        self.currentState = EXECUTE_STATE
                        self.trajectory_path.header.stamp = rospy.Time.now()
                        self.trajectory_pub.publish(self.trajectory_path)
                
                # Null stiffness matrix for transparency
                stiffness_mat = K_default * 0
                min_idx = 0

            else:
                # Do necessary calculations 
                min_idx, p_min, d_min = self.nearest_point_on_trajectory(p_current, 5)
                stiffness_mat, p_reference = self.get_tolerance_tunnel_model_update_paramaters(d_min, p_min, p_current)
                
                # Publish nearest point for visualisation (RVIZ)
                self.visualise_nearest_trajectory_point(p_reference, T_current)

            # Publish updated parameters
            self.publish_update_parameters(self.trajectory_path.poses[min_idx], stiffness_mat)



    def initialise_new_trajectory(self):
        self.trajectory_path: Path

        positions = []
        for poseMsg in self.trajectory_path.poses:
            posMsg = poseMsg.pose.position

            positions.append(np.asarray([posMsg.x, posMsg.y, posMsg.z]))
        
        self.trajectory_positions = np.vstack(positions)

        # Publish first pose for EFF honing
        honePose = deepcopy(self.trajectory_path)
        honePose.poses = honePose.poses[:1]
        honePose.header.stamp = rospy.Time.now()
        self.trajectory_pub.publish(honePose)

    def nearest_point_on_trajectory(self, pos: np.array, vicinity_idx: int):
        prev_idx = self.pn_idx 
        
        # pos is the current [x,y,z] position of the Franka EFF
        idx_l = (prev_idx - vicinity_idx) if (prev_idx - vicinity_idx) > 0 else 0 
        traj_in_vicinity = self.trajectory_positions[idx_l: prev_idx + vicinity_idx]
        delta = traj_in_vicinity - pos
        dist = np.sqrt(np.einsum('ij,ij->i', delta, delta))
        
        min_idx_vicinity = np.argmin(dist)
        d_min = dist[min_idx_vicinity]
        p_min = traj_in_vicinity[min_idx_vicinity]

        min_idx = idx_l + min_idx_vicinity
        self.pn_idx = min_idx
        return min_idx, p_min, d_min

    def visualise_nearest_trajectory_point(self, p_ref, trans_mat):
    
        br = tf2_ros.TransformBroadcaster()
        tf = se3_to_TransformStamped(trans_mat, PARENT_FRAME, "trajectory_reference")

        # Change translation to the nearest reference position on the trajectory
        x, y, z = p_ref
        referencePosition = Vector3(x,y,z)
        tf.transform.translation = referencePosition
        br.sendTransform(tf)

    def get_tolerance_tunnel_model_update_paramaters(self, d: np.double, p_min, p_current):

        # Equal stiffness of magnitude k is applied to the x, y and z axes.  
        if d < self.tunnel_radius:
            # Within inner constant force tunnel.
            k = self.stiffness_limits[MIN_IDX]
            p_eqm = p_current
            rospy.loginfo(f'{"Movement Free Zone": ^30} {d:.5f}m')

        elif d < 2 * self.tunnel_radius:
            # Inside fault tolerant region. Use fault tunnel stiffness.
            k = self.f_fault_tolerance_stiffness(d)
            p_eqm = p_min
            rospy.loginfo(f"{'Fault Tolerance': ^30} {d:.5f}m")

        else:
            # In fault zone.
            k = self.stiffness_limits[MAX_IDX]
            p_eqm = p_min 
            rospy.logwarn(f"{'Fault Zone': ^30} {d:.5f}m")

        # Stiffness matrix K, is 6x6
        K_stiffness = K_default * k

        return K_stiffness, p_eqm
    

    def get_distance_model_update_parameters(self, d: np.double, p_min, p_current):
        
        # Overall assistance 
        return K_default * self.stiffness_limits[MAX_IDX], p_min
            
    def publish_update_parameters(self, pose_update: PoseStamped, stiffness_update):
        
        stamp = rospy.Time.now()
        # Equilibrium position
        pose_update.header.stamp = stamp
        self.pEqm_pub.publish(pose_update)
        
        # Stiffness matrix
        new_stiffness = StiffnessConfig()
        new_stiffness.force = stiffness_update[:3, :3].reshape(-1)
        new_stiffness.headers.stamp = stamp
        new_stiffness.headers.frame_id = PARENT_FRAME
        self.stiffnessParams_pub.publish(new_stiffness)

if __name__ == "__main__":
    rospy.init_node("assistive_rehab")
    ctrl = TunnelTrajectoryController(tunnel_radius=0.02, k_min=100, k_max = 500)
    rospy.spin()