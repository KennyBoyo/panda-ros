#!/usr/bin/env python3
import rospy
import sys
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
import message_filters
from std_msgs.msg import String, Int8
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from dynamic_reconfigure.msg import Config, DoubleParameter, GroupState
from std_msgs.msg import Int16MultiArray
from franka_msgs.msg import FrankaState
import tf.transformations as tr
from mrn_panda.msg import ImpedanceParams



class equilibrium_publisher:

  def __init__(self):
    self.sub = rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.equilibrium_adjuster_callback)
    self.pub = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_pose", PoseStamped, queue_size=10)
    self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controller/equilibrium_stiffness", ImpedanceParams, queue_size=10)
    self.imp = rospy.Subscriber("/cartesian_impedance_equilibrium_controller/impedance_mode", Int8, self.impedance_mode_callback)
    # self.stiffness = rospy.Publisher("/cartesian_impedance_equilibrium_controllerdynamic_reconfigure_compliance_param_node/parameter_updates", Config, queue_size=10)
    self.position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
    self.robot_pose_eq = PoseStamped()
    self.robot_pose = PoseStamped()
    self.index = 0
    self.buffer_size = 20
    self.k = 0
    self.k_t = [0] * 3


    self.mag_thres_inc = 3
    self.mag_thres_dec = 3

    self.translation_lower_limit = 0
    self.translation_upper_limit = 150
    self.robot_pose_list = [None] * self.buffer_size  
    self.robot_pose.header.frame_id = "panda_link0"

    self.mode = 0


  def equilibrium_adjuster_callback(self, actual: FrankaState):
    self.robot_pose_list[self.index] = actual
    self.index += 1
    if (self.index == self.buffer_size):
      self.index = 0
    if self.robot_pose_list[self.index] != None:
      actual = self.robot_pose_list[self.index]
      self.robot_pose.header.stamp = rospy.Time.now()

      # Convert robot end effector coordinates to quaternion
      quat = tr.quaternion_from_matrix([[actual.O_T_EE[0], actual.O_T_EE[4], actual.O_T_EE[8], actual.O_T_EE[12]], \
        [actual.O_T_EE[1], actual.O_T_EE[5], actual.O_T_EE[9], actual.O_T_EE[13]], \
          [actual.O_T_EE[2], actual.O_T_EE[6], actual.O_T_EE[10], actual.O_T_EE[14]], \
            [actual.O_T_EE[3], actual.O_T_EE[7], actual.O_T_EE[11], actual.O_T_EE[15]]])
      # print(actual.dtheta)

      # Set end effector robot position
      self.robot_pose.pose.position.x = max([min([actual.O_T_EE[12],
                                            self.position_limits[0][1]]),
                                            self.position_limits[0][0]])
      
      self.robot_pose.pose.position.y = max([min([actual.O_T_EE[13],
                                        self.position_limits[1][1]]),
                                        self.position_limits[1][0]])
      
      self.robot_pose.pose.position.z = max([min([actual.O_T_EE[14],
                                        self.position_limits[2][1]]),
                                        self.position_limits[2][0]])
      
      # Set end effector orientation
      self.robot_pose.pose.orientation.x = quat[0]
      self.robot_pose.pose.orientation.y = quat[1]
      self.robot_pose.pose.orientation.z = quat[2]
      self.robot_pose.pose.orientation.w = quat[3]


      
      self.set_k(actual=actual)
      
      

      # stiffness_config.doubles.append(DoubleParameter(name="rotational_stiffness", value=20))
      # stiffness_config.doubles.append(DoubleParameter(name="nullspace_stiffness", value=10))
      # stiffness_config.groups.append(GroupState(name="Default", state=True, id=0, parent=0))
      # self.stiffness.publish(stiffness_config)

      # stiffness_config = Config()
      # stiffness_config.doubles.append(DoubleParameter(name="translational_stiffness", value=200))
      # stiffness_config.doubles.append(DoubleParameter(name="rotational_stiffness", value=20))
      # stiffness_config.doubles.append(DoubleParameter(name="nullspace_stiffness", value=10))
      # stiffness_config.groups.append(GroupState(name="Default", state=True, id=0, parent=0))
      # self.stiffness.publish(stiffness_config)

      # Publish pose
      self.pub.publish(self.robot_pose)


  def set_k(self, actual):
    stiffness_config = ImpedanceParams()
    stiffness_config.headers.stamp = rospy.Time.now()
    index_delay = 1
    magnitude = (20*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2 + (actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2 + (actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5)
    magnitudes = ((30*((actual.O_T_EE[12] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[12])**2)**0.5), (30*((actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2)**0.5), (30*((actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5))
    # magnitude_y = (20*((actual.O_T_EE[13] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[13])**2)**0.5)
    # magnitude_z = (20*((actual.O_T_EE[14] - self.robot_pose_list[(self.index-index_delay) % 20].O_T_EE[14])**2)**0.5)
    # rospy.logdebug(magnitude, logger_name="abcd")
    # # print(magnitude*100)
    # print(((actual.O_T_EE[12] - self.robot_pose_list[(self.index-10) % 20].O_T_EE[12])**2 + (actual.O_T_EE[13] - self.robot_pose_list[(self.index-10) % 20].O_T_EE[13])**2 + (actual.O_T_EE[14] - self.robot_pose_list[(self.index-10) % 20].O_T_EE[14])**2)**0.5)
    # magnitude -= 3


    #SIGMOID
    for i in range(len(magnitudes)):
      if (magnitudes[i] < self.mag_thres_dec):
        self.k_t[i] -= 1
      elif (magnitudes[i] > self.mag_thres_inc):
        self.k_t[i] += magnitudes[i]

      if (self.k_t[i] < 0):
          self.k_t[i] = 0

    if (magnitude < self.mag_thres_dec):
        self.k -= 1
    elif (magnitude > self.mag_thres_inc):
      self.k += magnitude

    if (self.k < 0):
        self.k = 0

    # translational_stiffness = DoubleParameter(name="translational_stiffness", value=self.k)
    # rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.k)
    # nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k)

    translation_stiffness_labels = ["translational_stiffness_x", "translational_stiffness_y", "translational_stiffness_z"]

    if (self.mode == 0):
        stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=0))
        rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=0)
        nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=0)
  
    else:
      for i in range(len(magnitudes)):
        if (self.k_t[i] < self.translation_lower_limit):
          stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=0))
        elif (self.k_t[i] > self.translation_upper_limit):
          self.k_t[i] = self.translation_upper_limit
          stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=self.translation_upper_limit))
        else:
          stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=self.k_t[i]))


      if (self.k < self.translation_lower_limit):
        rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=0)
        nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=0)
      elif (self.k > self.translation_upper_limit):
        self.k = self.translation_upper_limit
        rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.translation_upper_limit/3)
        nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)
      else:
        rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.k/3)
        nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)
      # for i in range(len(magnitudes)):
      #   if (self.k_t[i] < self.translation_lower_limit):
      #     stiffness_config.data.append(DoubleParameter(name=translation_stiffness_labels[i], value=0))
      #   elif (self.k_t[i] > self.translation_upper_limit):
      #     self.k_t[i] = self.translation_upper_limit
      #     stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=self.translation_upper_limit))
      #   else:
      #     stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=self.k_t[i]))


      # if (self.k < self.translation_lower_limit):
      #   stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=0))
      #   rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=0)
      #   nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=0)
      # elif (self.k > self.translation_upper_limit):
      #   self.k = self.translation_upper_limit
      #   stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=self.translation_upper_limit))
      #   rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.translation_upper_limit/3)
      #   nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)
      # else:
      #   stiffness_config.data.append(DoubleParameter(name="translational_stiffness", value=self.k))
      #   rotational_stiffness = DoubleParameter(name="rotational_stiffness", value=self.k/3)
      #   nullspace_stiffness = DoubleParameter(name="nullspace_stiffness", value=self.k/2)
    
    
    stiffness_config.data.append(rotational_stiffness)
    stiffness_config.data.append(nullspace_stiffness)
    # rospy.loginfo(self.mode)

    stiffness_config.data.append(DoubleParameter(name="mode", value=self.mode))

    self.stiffness.publish(stiffness_config)


  def impedance_mode_callback(self, msg):
    rospy.loginfo(self.mode)
    self.mode = msg.data


def main(args):
  rospy.init_node('equilibrium_publisher', anonymous=True, log_level=rospy.DEBUG)
  obc = equilibrium_publisher()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)


# header: 
#   seq: 22645
#   stamp: 
#     secs: 1678670276
#     nsecs: 308802217
#   frame_id: ''
# cartesian_collision: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# cartesian_contact: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# q: [-0.5479809054646652, -0.41348938689472414, 0.27574316111377367, -2.0670964302462393, 0.2531613536212271, 2.120395054799223, 0.2308171068314049]
# q_d: [-0.5849885725953001, -0.5698680417707199, 0.5740232489198838, -2.3800694113677956, 0.2396344585749838, 1.7893205195662993, 0.22215958491742221]
# dq: [0.3402523349546736, 0.030748417707146793, 0.4227238789875984, -0.044108573417712425, -0.0012203180246195813, -0.2332690474473558, -0.001958657155023184]
# dq_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ddq_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# theta: [-0.5478324890136719, -0.4146158695220947, 0.27566275000572205, -2.065460681915283, 0.25322669744491577, 2.1206231117248535, 0.2308257669210434]
# dtheta: [0.3585834503173828, 0.025059711188077927, 0.4155650734901428, -0.0507073737680912, 0.0, -0.22657285630702972, 0.0]
# tau_J: [-0.36757123470306396, -16.278053283691406, -4.111146450042725, 23.606143951416016, 0.5880944132804871, 3.0407934188842773, 0.07794080674648285]
# dtau_J: [-92.36146545410156, -6.170707702636719, 75.71533966064453, 118.08282470703125, 22.31476593017578, 4.3594584465026855, 15.462023735046387]
# tau_J_d: [0.0446469930807948, -0.4959635895901, -0.018271589139327714, -0.0029910895508430757, -0.0087237573749391, -0.021982639346154534, -0.0019341549740867177]
# K_F_ext_hat_K: [-2.240149389720543, 8.724782043009078, -4.439702343417989, 0.9065766357011801, 0.21311673855750796, 0.088513693787384]
# elbow: [0.27574316111377367, -1.0]
# elbow_d: [0.5740232489198838, -1.0]
# elbow_c: [0.0, 0.0]
# delbow_c: [0.0, 0.0]
# ddelbow_c: [0.0, 0.0]
# joint_collision: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# joint_contact: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_F_ext_hat_K: [-2.443490105747798, -8.927113427879597, 3.896856121943282, 5.4089425505714255, -3.352191902154056, -4.4327234781523455]
# O_dP_EE_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_ddP_O: [0.0, 0.0, -9.81]
# O_dP_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_ddP_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# tau_ext_hat_filtered: [-4.558327201775514, -0.9112616507430564, -5.3263232684169175, 1.445859958138458, -0.2864029025999628, 0.6484189198420638, 0.045128896708814835]
# m_ee: 0.7300000190734863
# F_x_Cee: [-0.009999999776482582, 0.0, 0.029999999329447746]
# I_ee: [0.0010000000474974513, 0.0, 0.0, 0.0, 0.0024999999441206455, 0.0, 0.0, 0.0, 0.0017000000225380063]
# m_load: 0.0
# F_x_Cload: [0.0, 0.0, 0.0]
# I_load: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# m_total: 0.7300000190734863
# F_x_Ctotal: [-0.009999999776482582, 0.0, 0.029999999329447746]
# I_total: [0.0010000000474974513, 0.0, 0.0, 0.0, 0.0024999999441206455, 0.0, 0.0, 0.0, 0.0017000000225380063]
# O_T_EE: [0.872937457849852, 0.20518296844909387, 0.44256173698396495, 0.0, 0.17565011984077464, -0.9785850850596528, 0.10723345373493176, 0.0, 0.45509555538467045, -0.01587238195419409, -0.8903011304947435, 0.0, 0.5093806920754759, -0.0944111768356032, 0.5497567112769909, 1.0]
# O_T_EE_d: [0.9160220523627841, 0.3865229521204091, -0.10716507812904831, 0.0, 0.3900931994428273, -0.9206617549729793, 0.013783162907715748, 0.0, -0.09333707711342587, -0.05443109733797179, -0.9941455857561842, 0.0, 0.3862299820618609, 0.027721423391134724, 0.3809602251595623, 1.0]
# O_T_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# F_T_EE: [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0]
# F_T_NE: [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0]
# NE_T_EE: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
# EE_T_K: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
# time: 5268.55
# control_command_success_rate: 0.99
# robot_mode: 2
# current_errors: 
#   joint_position_limits_violation: False
#   cartesian_position_limits_violation: False
#   self_collision_avoidance_violation: False
#   joint_velocity_violation: False
#   cartesian_velocity_violation: False
#   force_control_safety_violation: False
#   joint_reflex: False
#   cartesian_reflex: False
#   max_goal_pose_deviation_violation: False
#   max_path_pose_deviation_violation: False
#   cartesian_velocity_profile_safety_violation: False
#   joint_position_motion_generator_start_pose_invalid: False
#   joint_motion_generator_position_limits_violation: False
#   joint_motion_generator_velocity_limits_violation: False
#   joint_motion_generator_velocity_discontinuity: False
#   joint_motion_generator_acceleration_discontinuity: False
#   cartesian_position_motion_generator_start_pose_invalid: False
#   cartesian_motion_generator_elbow_limit_violation: False
#   cartesian_motion_generator_velocity_limits_violation: False
#   cartesian_motion_generator_velocity_discontinuity: False
#   cartesian_motion_generator_acceleration_discontinuity: False
#   cartesian_motion_generator_elbow_sign_inconsistent: False
#   cartesian_motion_generator_start_elbow_invalid: False
#   cartesian_motion_generator_joint_position_limits_violation: False
#   cartesian_motion_generator_joint_velocity_limits_violation: False
#   cartesian_motion_generator_joint_velocity_discontinuity: False
#   cartesian_motion_generator_joint_acceleration_discontinuity: False
#   cartesian_position_motion_generator_invalid_frame: False
#   force_controller_desired_force_tolerance_violation: False
#   controller_torque_discontinuity: False
#   start_elbow_sign_inconsistent: False
#   communication_constraints_violation: False
#   power_limit_violation: False
#   joint_p2p_insufficient_torque_for_planning: False
#   tau_j_range_violation: False
#   instability_detected: False
#   joint_move_in_wrong_direction: False
#   cartesian_spline_motion_generator_violation: False
#   joint_via_motion_generator_planning_joint_limit_violation: False
#   base_acceleration_initialization_timeout: False
#   base_acceleration_invalid_reading: False
# last_motion_errors: 
#   joint_position_limits_violation: False
#   cartesian_position_limits_violation: False
#   self_collision_avoidance_violation: False
#   joint_velocity_violation: False
#   cartesian_velocity_violation: False
#   force_control_safety_violation: False
#   joint_reflex: False
#   cartesian_reflex: True
#   max_goal_pose_deviation_violation: False
#   max_path_pose_deviation_violation: False
#   cartesian_velocity_profile_safety_violation: False
#   joint_position_motion_generator_start_pose_invalid: False
#   joint_motion_generator_position_limits_violation: False
#   joint_motion_generator_velocity_limits_violation: False
#   joint_motion_generator_velocity_discontinuity: False
#   joint_motion_generator_acceleration_discontinuity: False
#   cartesian_position_motion_generator_start_pose_invalid: False
#   cartesian_motion_generator_elbow_limit_violation: False
#   cartesian_motion_generator_velocity_limits_violation: False
#   cartesian_motion_generator_velocity_discontinuity: False
#   cartesian_motion_generator_acceleration_discontinuity: False
#   cartesian_motion_generator_elbow_sign_inconsistent: False
#   cartesian_motion_generator_start_elbow_invalid: False
#   cartesian_motion_generator_joint_position_limits_violation: False
#   cartesian_motion_generator_joint_velocity_limits_violation: False
#   cartesian_motion_generator_joint_velocity_discontinuity: False
#   cartesian_motion_generator_joint_acceleration_discontinuity: False
#   cartesian_position_motion_generator_invalid_frame: False
#   force_controller_desired_force_tolerance_violation: False
#   controller_torque_discontinuity: False
#   start_elbow_sign_inconsistent: False
#   communication_constraints_violation: False
#   power_limit_violation: False
#   joint_p2p_insufficient_torque_for_planning: False
#   tau_j_range_violation: False
#   instability_detected: False
#   joint_move_in_wrong_direction: False
#   cartesian_spline_motion_generator_violation: False
#   joint_via_motion_generator_planning_joint_limit_violation: False
#   base_acceleration_initialization_timeout: False
#   base_acceleration_invalid_reading: False


# header: 
#   seq: 24791
#   stamp: 
#     secs: 1678670349
#     nsecs: 299793149
#   frame_id: ''
# cartesian_collision: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# cartesian_contact: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# q: [-0.49309429442407876, -0.4137882349446503, 0.4422840566802431, -2.0783002347108166, 0.24458810145987403, 2.1046491799884373, 0.2353660047713058]
# q_d: [-0.5849885725953001, -0.5698680417707199, 0.5740232489198838, -2.3800694113677956, 0.2396344585749838, 1.7893205195662993, 0.22215958491742221]
# dq: [-0.0007727517079145006, 0.0003828299650502479, -0.00126937803049443, 0.0005751077944119342, -6.416275129633213e-05, -7.398869613140607e-05, 0.0012014260386369571]
# dq_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ddq_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# theta: [-0.49308860301971436, -0.4147500991821289, 0.44191116094589233, -2.076690435409546, 0.2446603626012802, 2.104957342147827, 0.2353701889514923]
# dtheta: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# tau_J: [0.08073198795318604, -13.711219787597656, -5.325135707855225, 22.939640045166016, 0.6503502726554871, 2.7734594345092773, 0.03765760362148285]
# dtau_J: [93.8817138671875, 38.009029388427734, 59.80998229980469, -24.232643127441406, 19.333038330078125, -4.5386643409729, -36.41016387939453]
# tau_J_d: [2.86074080877634e-07, -5.851409013491346e-07, 2.2540635697327322e-07, -6.90285372051455e-07, -2.7855102529733746e-08, -1.4335023516198243e-07, 1.900756623389869e-09]
# K_F_ext_hat_K: [0.6364294661464246, -0.19677010165984524, -0.7949066363185346, -0.17596390884047372, -0.22230364167034727, 0.0773805037714703]
# elbow: [0.4422840566802431, -1.0]
# elbow_d: [0.5740232489198838, -1.0]
# elbow_c: [0.0, 0.0]
# delbow_c: [0.0, 0.0]
# ddelbow_c: [0.0, 0.0]
# joint_collision: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# joint_contact: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_F_ext_hat_K: [0.13981799645591061, 0.4230341161923566, 0.9365479269002298, -0.40023071296440405, -0.27959268459114195, 0.04750019274802415]
# O_dP_EE_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_ddP_O: [0.0, 0.0, -9.81]
# O_dP_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# O_ddP_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# tau_ext_hat_filtered: [0.04764668045920271, -0.41015448043153996, 0.07306251477643912, 0.42320060100052315, -0.12165401320922009, 0.06494826932107353, 0.07745366389273972]
# m_ee: 0.7300000190734863
# F_x_Cee: [-0.009999999776482582, 0.0, 0.029999999329447746]
# I_ee: [0.0010000000474974513, 0.0, 0.0, 0.0, 0.0024999999441206455, 0.0, 0.0, 0.0, 0.0017000000225380063]
# m_load: 0.0
# F_x_Cload: [0.0, 0.0, 0.0]
# I_load: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# m_total: 0.7300000190734863
# F_x_Ctotal: [-0.009999999776482582, 0.0, 0.029999999329447746]
# I_total: [0.0010000000474974513, 0.0, 0.0, 0.0, 0.0024999999441206455, 0.0, 0.0, 0.0, 0.0017000000225380063]
# O_T_EE: [0.835381687483237, 0.4045683902099455, 0.37207875612805535, 0.0, 0.37753078990567157, -0.914318882725048, 0.14653405838516825, 0.0, 0.39948937210889435, 0.018059665445098395, -0.9165599216941862, 0.0, 0.5195488693237763, 0.023711876191656444, 0.5257891203650447, 1.0]
# O_T_EE_d: [0.9160220523627841, 0.3865229521204091, -0.10716507812904831, 0.0, 0.3900931994428273, -0.9206617549729793, 0.013783162907715748, 0.0, -0.09333707711342587, -0.05443109733797179, -0.9941455857561842, 0.0, 0.3862299820618609, 0.027721423391134724, 0.3809602251595623, 1.0]
# O_T_EE_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# F_T_EE: [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0]
# F_T_NE: [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.10339999943971634, 1.0]
# NE_T_EE: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
# EE_T_K: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
# time: 5341.542
# control_command_success_rate: 0.99
# robot_mode: 2
# current_errors: 
#   joint_position_limits_violation: False
#   cartesian_position_limits_violation: False
#   self_collision_avoidance_violation: False
#   joint_velocity_violation: False
#   cartesian_velocity_violation: False
#   force_control_safety_violation: False
#   joint_reflex: False
#   cartesian_reflex: False
#   max_goal_pose_deviation_violation: False
#   max_path_pose_deviation_violation: False
#   cartesian_velocity_profile_safety_violation: False
#   joint_position_motion_generator_start_pose_invalid: False
#   joint_motion_generator_position_limits_violation: False
#   joint_motion_generator_velocity_limits_violation: False
#   joint_motion_generator_velocity_discontinuity: False
#   joint_motion_generator_acceleration_discontinuity: False
#   cartesian_position_motion_generator_start_pose_invalid: False
#   cartesian_motion_generator_elbow_limit_violation: False
#   cartesian_motion_generator_velocity_limits_violation: False
#   cartesian_motion_generator_velocity_discontinuity: False
#   cartesian_motion_generator_acceleration_discontinuity: False
#   cartesian_motion_generator_elbow_sign_inconsistent: False
#   cartesian_motion_generator_start_elbow_invalid: False
#   cartesian_motion_generator_joint_position_limits_violation: False
#   cartesian_motion_generator_joint_velocity_limits_violation: False
#   cartesian_motion_generator_joint_velocity_discontinuity: False
#   cartesian_motion_generator_joint_acceleration_discontinuity: False
#   cartesian_position_motion_generator_invalid_frame: False
#   force_controller_desired_force_tolerance_violation: False
#   controller_torque_discontinuity: False
#   start_elbow_sign_inconsistent: False
#   communication_constraints_violation: False
#   power_limit_violation: False
#   joint_p2p_insufficient_torque_for_planning: False
#   tau_j_range_violation: False
#   instability_detected: False
#   joint_move_in_wrong_direction: False
#   cartesian_spline_motion_generator_violation: False
#   joint_via_motion_generator_planning_joint_limit_violation: False
#   base_acceleration_initialization_timeout: False
#   base_acceleration_invalid_reading: False
# last_motion_errors: 
#   joint_position_limits_violation: False
#   cartesian_position_limits_violation: False
#   self_collision_avoidance_violation: False
#   joint_velocity_violation: False
#   cartesian_velocity_violation: False
#   force_control_safety_violation: False
#   joint_reflex: False
#   cartesian_reflex: True
#   max_goal_pose_deviation_violation: False
#   max_path_pose_deviation_violation: False
#   cartesian_velocity_profile_safety_violation: False
#   joint_position_motion_generator_start_pose_invalid: False
#   joint_motion_generator_position_limits_violation: False
#   joint_motion_generator_velocity_limits_violation: False
#   joint_motion_generator_velocity_discontinuity: False
#   joint_motion_generator_acceleration_discontinuity: False
#   cartesian_position_motion_generator_start_pose_invalid: False
#   cartesian_motion_generator_elbow_limit_violation: False
#   cartesian_motion_generator_velocity_limits_violation: False
#   cartesian_motion_generator_velocity_discontinuity: False
#   cartesian_motion_generator_acceleration_discontinuity: False
#   cartesian_motion_generator_elbow_sign_inconsistent: False
#   cartesian_motion_generator_start_elbow_invalid: False
#   cartesian_motion_generator_joint_position_limits_violation: False
#   cartesian_motion_generator_joint_velocity_limits_violation: False
#   cartesian_motion_generator_joint_velocity_discontinuity: False
#   cartesian_motion_generator_joint_acceleration_discontinuity: False
#   cartesian_position_motion_generator_invalid_frame: False
#   force_controller_desired_force_tolerance_violation: False
#   controller_torque_discontinuity: False
#   start_elbow_sign_inconsistent: False
#   communication_constraints_violation: False
#   power_limit_violation: False
#   joint_p2p_insufficient_torque_for_planning: False
#   tau_j_range_violation: False
#   instability_detected: False
#   joint_move_in_wrong_direction: False
#   cartesian_spline_motion_generator_violation: False
#   joint_via_motion_generator_planning_joint_limit_violation: False
#   base_acceleration_initialization_timeout: False
#   base_acceleration_invalid_reading: False