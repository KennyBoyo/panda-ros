#!/usr/bin/env python3

import rospy
import numpy as np 
import tf2_ros
from copy import deepcopy 
from collections import deque
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from panda_ros.msg import StiffnessConfig, TrainingStatus
from geometry_msgs.msg import PoseStamped, Vector3
from std_srvs.srv import Trigger, TriggerResponse, SetBoolResponse, SetBool, SetBoolRequest
from panda_ros.srv import *
from visualization_msgs.msg import Marker
from msg_utils import *
import os

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
RAMPING_UP_STATE = 5
RAMPING_DOWN_STATE = 6

# Assistance Types:
NO_ASSIST = 0
DISTANCE_ASSIST = 1
TUNNEL_ASSIST = 2
assistance_mode_repr = ["NO_ASSIST", "CONSTANT_ASSIST", "TUNNEL_ASSIST"]

# Rotational Stiffness
FREE_ROTATIONAL_STIFFNESS = 0
TRAINING_ROTATIONAL_STIFFNESS = 15


class StiffnessRamp():
    def __init__(self, initialStiffness, desiredStiffness, desired_pose = PoseStamped(), steps_target = 100):
        self.K_initial = initialStiffness
        self.K_desired = desiredStiffness
        self.desired_pose = desired_pose

        self.K_delta = desiredStiffness - initialStiffness


        self.step_target = steps_target
        self.step_current = 0

        if np.array_equal(initialStiffness, desiredStiffness):
            self.step_current = self.step_target
    
    def get_new_stiffness(self):
        factor = self.step_current/self.step_target
        self.step_current += 1
        return self.K_initial + self.K_delta * factor 
    
    def get_desired_pose(self):
        return self.desired_pose
    
    def get_completion_status(self):
        if self.step_current == self.step_target:
            return True
        else:
            return False
    


class TunnelTrajectoryController():
    def __init__ (self, tunnel_radius: np.double, ktunnel_min, ktunnel_max, kdistance, recordFlag: bool, record_path: str):
        self.tunnel_radius = tunnel_radius
        self.hone_dist = tunnel_radius/2
        self.record_data = recordFlag

        self.stiffness_limits = (ktunnel_min, ktunnel_max)
        self.stiffness_distance = kdistance
        self.currentState = STARTING_STATE
        self.previousStiffness = np.zeros((6,6))
        self.pn_idx = 0
        self.trajectory_id = 0

        self.recordPath = record_path
        self.f_fault_tolerance_stiffness = lambda d : ktunnel_min + ((ktunnel_max - ktunnel_min)/tunnel_radius)*(d-tunnel_radius)
        self.setup_marker_message()

        self.pbServices = [rospy.Service('assistive_controller/next_trajectory', Trigger, self.pb_next_trajectory_handler),
                           rospy.Service('assistive_controller/reset_trajectory', Trigger, self.pb_reset_trajectory_handler),
                           rospy.Service('assistive_controller/no_assist', SetBool, self.pb_no_assist_handler),
                           rospy.Service('assistive_controller/distance_assist', SetBool, self.pb_distance_assist_handler),
                           rospy.Service('assistive_controller/tunnel_assist', SetBool, self.pb_tunnel_assist_handler)]
        
        self.assistanceMode = NO_ASSIST

        self.pbAssistanceSelection = np.full((3,1), False)
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

            self.nextState_afterRamp = NEW_TRAJECTORY_STATE
            self.currentState = RAMPING_DOWN_STATE

            self.currentRamp = StiffnessRamp(self.previousStiffness, np.zeros((6,6)))
            rospy.loginfo("Ramping down then Reset trajectory")

        elif self.pbNextTrajectoryState:
            self.pbNextTrajectoryState = False
            # Reset to IDLE state (Remove path?)
            self.currentRamp = StiffnessRamp(self.previousStiffness, np.zeros((6,6)))

            self.nextState_afterRamp = IDLE_STATE
            self.currentState = RAMPING_DOWN_STATE

            rospy.loginfo("Ramping down then Next trajectory")

        # Check for state change in assistance_type

        if state != EXECUTE_STATE and state != RAMPING_DOWN_STATE and state != RAMPING_UP_STATE:
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
                    self.honing_pose = self.initialise_new_trajectory()
                    self.currentState = HONING_STATE
                    
            elif state == HONING_STATE:
                p_delta = np.linalg.norm(p_current - self.trajectory_positions[0])
                # desired_stiffness = K_default * self.stiffness_limits[MIN_IDX]
                desired_pose = self.honing_pose
                rospy.logwarn_throttle(2 ,f"Please hone EFF: {p_delta*1e3} millimeters away. Control pose of EFF.")

                if p_delta < self.hone_dist:
                    self.set_assistance_mode()
                    rospy.loginfo(f"EFF honed. Ramping up.")
                    self.currentState = RAMPING_UP_STATE

                    if self.assistanceMode == NO_ASSIST:
                        self.currentRamp = StiffnessRamp(np.zeros((6,6)), np.zeros((6,6)), self.honing_pose)
                    
                    elif self.assistanceMode == TUNNEL_ASSIST:
                        desired_ramp = np.diag([1,1,1,0,0,0]) * np.mean(self.stiffness_limits)
                        desired_ramp[3:, 3:] = np.eye(3) * TRAINING_ROTATIONAL_STIFFNESS
                        self.currentRamp = StiffnessRamp(np.zeros((6,6)), desired_ramp, self.honing_pose)
                    
                    elif self.assistanceMode == DISTANCE_ASSIST:
                        desired_ramp = np.diag([1,1,1,0,0,0]) * self.stiffness_distance
                        desired_ramp[3:, 3:] = np.eye(3) * TRAINING_ROTATIONAL_STIFFNESS
                        self.currentRamp = StiffnessRamp(np.zeros((6,6)), desired_ramp, self.honing_pose)

        elif state == RAMPING_UP_STATE:
            desired_pose = self.currentRamp.get_desired_pose()

            if self.currentRamp.get_completion_status():
                # Publish desired trajectory once.
                self.trajectory_path.header.stamp = rospy.Time.now()
                self.trajectory_pub.publish(self.trajectory_path)

                self.currentState = EXECUTE_STATE
                desired_stiffness = self.currentRamp.K_desired
                rospy.loginfo(f"Ramp up complete. Starting assistive rehabilitation in {assistance_mode_repr[self.assistanceMode]} mode.")
            else: 
                desired_stiffness = self.currentRamp.get_new_stiffness()

        elif state == RAMPING_DOWN_STATE:
            desired_pose = se3_to_PoseStamped(T_current, PARENT_FRAME)
            if self.currentRamp.get_completion_status():

                self.currentState = self.nextState_afterRamp
                desired_stiffness = self.currentRamp.K_desired
                desired_pose = desired_pose
                rospy.loginfo("Ramp down complete.")
            else: 
                desired_stiffness = self.currentRamp.get_new_stiffness()
        else:
            # Do necessary calculations 
            min_idx, p_min, d_min = self.nearest_point_on_trajectory(p_current, 5)
            deviation_ratio = d_min/(2*self.tunnel_radius)

            # Get desired stiffness based on assistance mode. 
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
            desired_stiffness[3:, 3:] = np.eye(3) * TRAINING_ROTATIONAL_STIFFNESS

            if self.record_data:
                status_msg = TrainingStatus()
                status_msg.header.stamp = rospy.Time.now()
                status_msg.assistance_mode = assistance_mode_repr[self.assistanceMode]
                status_msg.d_nearest = d_min
                status_msg.k_value = desired_stiffness[0,0]
                status_msg.p_current = Vector3(*p_current)
                status_msg.p_nearest = Vector3(*p_min)
                status_msg.trajectory_id = self.trajectory_id
                self.status_pub.publish(status_msg)

        # Publish updated parameters
        self.publish_update_parameters(desired_pose, desired_stiffness)

    def pb_next_trajectory_handler(self, req):
        self.pbNextTrajectoryState = True
        return TriggerResponse(success=True)
    
    def pb_no_assist_handler(self, req: SetBoolRequest):
        self.pbAssistanceSelection[0] = req.data
        return SetBoolResponse(success=True)
    def pb_distance_assist_handler(self, req: SetBoolRequest):
        self.pbAssistanceSelection[1] = req.data
        return SetBoolResponse(success=True)
    def pb_tunnel_assist_handler(self, req: SetBoolRequest):
        self.pbAssistanceSelection[2] = req.data
        return SetBoolResponse(success=True)
    def pb_reset_trajectory_handler(self, req):
        self.pbResetTrajectoryState = True
        return TriggerResponse(success=True)
    
    def get_average_position(self, p_delta):
        self.honing_queue.append(p_delta)
        if np.all(self.honing_queue):
            return np.mean(self.honing_queue)

    def set_assistance_mode(self):
        
        if self.pbAssistanceSelection.sum() != 1:
            rospy.logerr("No/greater than one mode selected. Using NO_ASSIST.")
            self.assistanceMode = NO_ASSIST
        
        elif self.pbAssistanceSelection[0]:
            self.assistanceMode = NO_ASSIST

        elif self.pbAssistanceSelection[1]:
                self.assistanceMode = DISTANCE_ASSIST

        elif self.pbAssistanceSelection[2]:
                self.assistanceMode = TUNNEL_ASSIST
    
    def initialise_new_trajectory(self):
        self.trajectory_path: Path

        positions = []
        for poseMsg in self.trajectory_path.poses:
            posMsg = poseMsg.pose.position

            positions.append(np.asarray([posMsg.x, posMsg.y, posMsg.z]))
        
        self.trajectory_positions = np.vstack(positions)
        self.trajectory_id = np.random.randint(1000)
        
        # Save new desired path in the paths folder of the directory specified
        np.save(os.path.join(self.recordPath, "path_data", "{self.trajectory_id}.npy"), self.trajectory_positions)

        # Publish first pose for EFF honing
        honePose = deepcopy(self.trajectory_path)
        honePose.poses = honePose.poses[:1]
        honePose.header.stamp = rospy.Time.now()
        self.trajectory_pub.publish(honePose)
        self.deviation_pub.publish(Float64(-1))

        
        desired_pose = honePose.poses[0]
        self.marker.pose = desired_pose.pose
        self.marker.header.stamp = rospy.Time.now()
        self.pHoning_pub.publish(self.marker)

        return desired_pose

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
            rospy.loginfo_throttle(2, f'{"Movement Free Zone": ^30} {d:.5f}m')

        elif d < 2 * self.tunnel_radius:
            # Inside fault tolerant region. Use fault tunnel stiffness.
            k = self.f_fault_tolerance_stiffness(d)
            p_eqm = p_min
            deviation_ratio = (d-self.tunnel_radius)/(self.tunnel_radius)
            rospy.loginfo_throttle(2, f"{'Fault Tolerance': ^30} {d:.5f}m")

        else:
            # In fault zone.
            k = self.stiffness_limits[MAX_IDX]
            p_eqm = p_min 
            deviation_ratio = 1
            rospy.logwarn_throttle(2, f"{'Fault Zone': ^30} {d:.5f}m")

        # Stiffness matrix K, is 6x6
        K_stiffness = K_default * k

        return K_stiffness, p_eqm, deviation_ratio
    

    def get_distance_model_update_parameters(self, d: np.double, p_min, p_current):
        
        # Overall assistance 
        return K_default * self.stiffness_distance, p_min
            
    def publish_update_parameters(self, pose_update: PoseStamped, stiffness_update):
        
        stamp = rospy.Time.now()
        # Equilibrium position
        pose_update.header.stamp = stamp
        self.pEqm_pub.publish(pose_update)
        
        self.previousStiffness = stiffness_update

        # Stiffness matrix
        new_stiffness = StiffnessConfig()
        new_stiffness.force = stiffness_update[:3, :3].reshape(-1)
        new_stiffness.headers.stamp = stamp
        new_stiffness.headers.frame_id = PARENT_FRAME
        new_stiffness.torque = stiffness_update[3:, 3:].reshape(-1)

        self.stiffnessParams_pub.publish(new_stiffness)

if __name__ == "__main__":
    rospy.init_node("rehabilitation_train")
    
    recordFlag = rospy.get_param('~record_bag')
    KTunnel_min = rospy.get_param('~k_tunnel_min')
    KTunnel_max = rospy.get_param('~k_tunnel_max')
    RTunnel = rospy.get_param('~r_tunnel')
    KDistance = rospy.get_param('~k_distance')
    recordPath = rospy.get_param('~record_path')

    ctrl = TunnelTrajectoryController(tunnel_radius = RTunnel, 
                                        ktunnel_min = KTunnel_min,
                                        ktunnel_max = KTunnel_max,
                                        kdistance = KDistance,
                                        recordFlag=recordFlag,
                                        record_path=recordPath)
    rospy.spin()