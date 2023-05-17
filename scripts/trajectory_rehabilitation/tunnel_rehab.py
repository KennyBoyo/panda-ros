#!/usr/bin/env python3

import rospy
import numpy as np 
import tf2_ros
from copy import deepcopy 
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from panda_ros.msg import StiffnessConfig, TrainingStatus
from geometry_msgs.msg import PoseStamped, Vector3
from std_srvs.srv import Trigger, TriggerResponse, SetBoolResponse, SetBool, SetBoolRequest
from panda_ros.srv import *
from visualization_msgs.msg import Marker
from msg_utils import *

PARENT_FRAME = "panda_link0"

# Honing Parameters
# hone_dist being used as tunnel_radius

# Constants
K_default =  np.diag([1,1,1,0,0,0])
MIN_IDX = 0
MAX_IDX = 1

# States
STARTING_STATE = -1
IDLE_STATE = 0
NEW_TRAJECTORY_STATE = 1
HONING_STATE = 2
EXECUTE_STATE = 3
RESET_STATE = 4

# Assistance Types:
NO_ASSIST = 0
DISTANCE_ASSIST = 1
TUNNEL_ASSIST = 2
assistance_mode_repr = ["NO_ASSIST", "DISTANCE_ASSIST", "TUNNEL_ASSIST"]
class TunnelTrajectoryController():
    def __init__ (self, tunnel_radius: np.double, k_min, k_max, recordFlag: bool):
        self.tunnel_radius = tunnel_radius
        self.hone_dist = 0.05
        self.record_data = recordFlag

        self.stiffness_limits = (k_min, k_max)
        self.currentState = STARTING_STATE
        self.pn_idx = 0
        self.f_fault_tolerance_stiffness = lambda d : k_min + ((k_max - k_min)/tunnel_radius)*(d-tunnel_radius)
        self.setup_marker_message()

        self.pbServices = [rospy.Service('assistive_controller/next_trajectory', Trigger, self.pb_next_trajectory_handler),
                           rospy.Service('assistive_controller/reset_trajectory', Trigger, self.pb_reset_trajectory_handler),
                           rospy.Service('assistive_controller/assistance_flag', SetBool, self.pbs_assistance_flag_handler),
                           rospy.Service('assistive_controller/assistance_type', SetBool, self.pbs_assistance_type_handler)]
        self.assistanceMode = TUNNEL_ASSIST
        # Assistance_flag(0th element) - ON: Assistance mode enabled.
        # Assistance_type(1st element) - OFF: Distance based, ON: Tunnel based.
        self.pbAssistanceStates = [True, True]
        self.pbNextTrajectoryState = False
        self.pbResetTrajectoryState = False
        
        # Wait for trajectory server    
        rospy.wait_for_service("/training_trajectory_server")
        # Incoming:
        self.pEFF_sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.step)
        self.trajectory_srv = rospy.ServiceProxy("training_trajectory_server", TrainingTrajectory)
        # Outgoing:
        self.pEqm_pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped, queue_size=10)
        self.stiffnessParams_pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/stiffness_config", StiffnessConfig, queue_size=10)
        self.trajectory_pub = rospy.Publisher("/unity/desired_trajectory", Path, queue_size=10)
        self.deviation_pub = rospy.Publisher("unity/eff_deviation", Float64, queue_size=10)
        self.pHoning_pub = rospy.Publisher("assistive_controller/honing_position", Marker, queue_size=10)
        self.status_pub = rospy.Publisher("assistive_controller/status", TrainingStatus, queue_size=10)

    def setup_marker_message(self):
        self.marker = Marker()
        self.marker.header.frame_id = PARENT_FRAME
        self.marker.id = 0
        self.marker.type = Marker().SPHERE
        self.marker.action = Marker().MODIFY
        self.marker.scale  = Vector3(0.05,0.05,0.05)
        self.marker.color.b = 1
        self.marker.color.a = 0.6

    def step(self, msg: FrankaState):
        state = self.currentState
        T_current = msg.O_T_EE
        p_current = np.array([T_current[12], T_current[13], T_current[14]])

        # Handle pb states
        if self.pbResetTrajectoryState:
            self.pbResetTrajectoryState = False
            self.currentState = NEW_TRAJECTORY_STATE
            rospy.loginfo("Reseting Trajectory....")

        elif self.pbNextTrajectoryState:
            self.pbNextTrajectoryState = False
            # Reset to IDLE state (Remove path?)
            self.currentState = IDLE_STATE

        # Check for state change in assistance_type

        if state != EXECUTE_STATE:
            # Null stiffness matrix for transparency
            desired_stiffness = K_default * 0
            desired_pose = PoseStamped()
            desired_pose.header.stamp = rospy.Time.now()
            desired_pose.header.frame_id = PARENT_FRAME

            if state == IDLE_STATE:
                try:
                    req = self.trajectory_srv(PARENT_FRAME)
                    req: TrainingTrajectoryResponse
                    if req.trajectoryAvailable:
                        rospy.loginfo("New Trajectory Received.")
                        self.trajectory_path = req.trajectory
                        self.currentState = NEW_TRAJECTORY_STATE
                    else:
                        rospy.logwarn_once("New Trajectory Unavailable.")

                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed: %s"%e)

            elif  state == NEW_TRAJECTORY_STATE:
                    self.initialise_new_trajectory()
                    self.currentState = HONING_STATE
                    
            elif state == HONING_STATE:
                p_delta = np.linalg.norm(p_current - self.trajectory_positions[0])
                # desired_stiffness = K_default * self.stiffness_limits[MIN_IDX]
                rospy.logwarn_throttle(2 ,f"Please hone EFF: {p_delta*1e3} millimeters away. Control pose of EFF.")
                
                if p_delta < self.hone_dist:
                    mode_repr = self.get_assistance_mode()
                    rospy.loginfo(f"EFF honed. Starting assistive rehabilitation in {mode_repr} mode.")
                    self.currentState = EXECUTE_STATE

                    # Publish desired trajectory once.
                    self.trajectory_path.header.stamp = rospy.Time.now()
                    self.trajectory_pub.publish(self.trajectory_path)
            
        else:
            # Do necessary calculations 
            min_idx, p_min, d_min = self.nearest_point_on_trajectory(p_current, 5)
            deviation_ratio = d_min/(2*self.tunnel_radius)

            # Get desired stiffness based on assitance mode. 
            if self.assistanceMode == DISTANCE_ASSIST:
                desired_stiffness, p_reference = self.get_distance_model_update_parameters(d_min, p_min, p_current)    
            elif self.assistanceMode == TUNNEL_ASSIST:
                desired_stiffness, p_reference, deviation_ratio = self.get_tolerance_tunnel_model_update_paramaters(d_min, p_min, p_current)
            else:
                desired_stiffness = K_default * 0
                p_reference = p_min

            if deviation_ratio > 1:
                deviation_ratio = 1
            self.deviation_pub.publish(Float64(deviation_ratio))

            desired_pose = self.trajectory_path.poses[min_idx]
            # Publish nearest point for visualisation (RVIZ)
            self.visualise_nearest_trajectory_point(p_reference, T_current)
            
            if self.record_data:
                status_msg = TrainingStatus()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.assistance_mode = assistance_mode_repr[self.assistanceMode]
                status_msg.d_nearest = d_min
                status_msg.k_value = desired_stiffness[0,0]
                status_msg.p_current = Vector3(*p_current)
                status_msg.p_nearest = Vector3(*p_min)
                self.status_pub.publish(status_msg)
        # Publish updated parameters
        self.publish_update_parameters(desired_pose, desired_stiffness)

    def pb_next_trajectory_handler(self, req):
        self.pbNextTrajectoryState = True
        return TriggerResponse(success=True)
    def pbs_assistance_flag_handler(self, req: SetBoolRequest):
        self.pbAssistanceStates[0] = req.data
        return SetBoolResponse(success=True)
    def pbs_assistance_type_handler(self, req: SetBoolRequest):
        self.pbAssistanceStates[1] = req.data
        return SetBoolResponse(success=True)
    def pb_reset_trajectory_handler(self, req):
        self.pbResetTrajectoryState = True
        return TriggerResponse(success=True)

    def get_assistance_mode(self):
        self.assistanceMode = NO_ASSIST
        mode_repr = "NO_ASSIST"
        if self.pbAssistanceStates[0]:

            if self.pbAssistanceStates[1]:
                self.assistanceMode = TUNNEL_ASSIST
                mode_repr = "TUNNEL_ASSIST"
            
            else:
                self.assistanceMode = DISTANCE_ASSIST
                mode_repr = "DISTANCE_ASSIST"

        return mode_repr
    
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
        
        self.marker.pose = honePose.poses[0].pose
        self.marker.header.stamp = rospy.Time.now()
        self.pHoning_pub.publish(self.marker)

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
            deviation_ratio = 0
            rospy.loginfo(f'{"Movement Free Zone": ^30} {d:.5f}m')

        elif d < 2 * self.tunnel_radius:
            # Inside fault tolerant region. Use fault tunnel stiffness.
            k = self.f_fault_tolerance_stiffness(d)
            p_eqm = p_min
            deviation_ratio = (d-self.tunnel_radius)/(self.tunnel_radius)
            rospy.loginfo(f"{'Fault Tolerance': ^30} {d:.5f}m")

        else:
            # In fault zone.
            k = self.stiffness_limits[MAX_IDX]
            p_eqm = p_min 
            deviation_ratio = 1
            rospy.logwarn(f"{'Fault Zone': ^30} {d:.5f}m")

        # Stiffness matrix K, is 6x6
        K_stiffness = K_default * k

        return K_stiffness, p_eqm, deviation_ratio
    

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
    recordFlag = rospy.get_param('~record_bag')

    ctrl = TunnelTrajectoryController(tunnel_radius=0.02, k_min=15, k_max = 400, recordFlag=recordFlag)
    rospy.spin()