// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_ros/cartesian_pose_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <ros/console.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <panda_ros/StiffnessConfig.h>
#include <panda_ros/PoseCommand.h>
#include <ros/ros.h>

#include <franka_panda_controller_swc/pseudo_inversion.h>

#define TRANSPARENCY_MODE 0
#define CARTESIAN_IMPEDANCE_MODE 1
#define CARTESIAN_POSE_MODE 2


namespace panda_ros {

bool CartesianPoseImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // Define Subscribers and Publishers
  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianPoseImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_equilibrium_stiffness_ = node_handle.subscribe(
      "stiffness_config", 20, &CartesianPoseImpedanceController::equilibriumStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_impedance_mode_= node_handle.subscribe(
      "impedance_mode", 20, &CartesianPoseImpedanceController::impedanceModeCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_pose_command_ = node_handle.subscribe(
      "pose_command", 20, &CartesianPoseImpedanceController::cartesianPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Get Panda Hardware Parameters and control

  ros::Duration desired_movement_time_;
  // Cartesian Impedance Interface
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianPoseImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianPoseImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface_ == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface_->getHandle(arm_id + "_robot"));
    
    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      q_start[i] = state_handle_->getRobotState().q_d[i];
    }

    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }

  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianPoseImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianPoseImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Cartesian Pose Interface
  cartesian_pose_interface_ = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  mode = TRANSPARENCY_MODE;

  std::cout << "CARTESIAN_POSE_IMPEDANCE_CONTROLLER" << std::endl;
  return true;
}

void CartesianPoseImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE_d.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  desired_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  elapsed_time_ = ros::Duration(0.0);
  desired_movement_time_ = ros::Duration(5.0);
  std::cout << "STARTED" << std::endl;
}

void CartesianPoseImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& period) {
  // std::cout << mode << std::endl;
  if (mode == TRANSPARENCY_MODE || mode == CARTESIAN_IMPEDANCE_MODE) {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE_d.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    franka_panda_controller_swc::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                        (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        0.5* filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  } else if (mode == CARTESIAN_POSE_MODE) {
    elapsed_time_ += period;

    std::array<double, 16> new_pose = initial_pose_;
    
    if (desired_movement_time_.toSec() < ros::Duration(1.0).toSec()) {
      desired_movement_time_ = ros::Duration(1.0);
    }
    if (elapsed_time_.toSec() > desired_movement_time_.toSec()) {
      elapsed_time_ = desired_movement_time_;
    }

    // Linearly approach desired pose
    for (auto i = 0; i < 16; i++) {
      new_pose[i] = initial_pose_[i] + ((desired_pose_[i] - initial_pose_[i]) * (elapsed_time_.toSec()/desired_movement_time_.toSec()));
    }
    
    cartesian_pose_handle_->setCommand(new_pose);
  }
}

Eigen::Matrix<double, 7, 1> CartesianPoseImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianPoseImpedanceController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianPoseImpedanceController::equilibriumStiffnessCallback(
    const panda_ros::StiffnessConfig& config) {

  if (mode == 0) {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    // nullspace_stiffness_target_ = config.data[5].value/5;
    nullspace_stiffness_target_ = 0;
  } else if (mode == 1) {
    Eigen::Matrix3d stiffness_tl = Eigen::Matrix3d::Zero(3, 3);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
        stiffness_tl.col(j).row(i) << config.force[3*i+j];
    }

    Eigen::Matrix3d stiffness_br = Eigen::Matrix3d::Zero(3, 3);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++)
        stiffness_br.col(j).row(i) << config.torque[3*i+j];
    }

    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << stiffness_tl;
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        // << stiffness_br;
        << 30 * Eigen::Matrix3d::Identity();
    #undef VERBOSE
    #ifdef VERBOSE
      std::cout << "cartesian_stiffness_target_" << std::endl;
      std::cout << cartesian_stiffness_target_ << std::endl;
    #endif

    cartesian_damping_target_.setIdentity();
    Eigen::Matrix3d damping_tl = Eigen::Matrix3d::Zero(3, 3);
    Eigen::Matrix3d damping_br = Eigen::Matrix3d::Zero(3, 3);
    for (int i = 0; i < 3; i++ ) {
        damping_tl.col(i).row(i) << 3.0 * sqrt(config.force_mag);
        damping_br.col(i).row(i) << 3.0 * sqrt(config.force_mag);
    }

    
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 15 * Eigen::Matrix3d::Identity();
        // << damping_tl;
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 15 * Eigen::Matrix3d::Identity();
    // nullspace_stiffness_target_ = config.data[4].value/5;
    #ifdef VERBOSE
      std::cout << "cartesian_damping_target_" << std::endl;
      std::cout << cartesian_damping_target_ << std::endl;
      #endif
    nullspace_stiffness_target_ = 10;
  }
  else {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 0 * Eigen::Matrix3d::Identity();
    // nullspace_stiffness_target_ = config.data[5].value/5;
    nullspace_stiffness_target_ = 0;

  }
}

void CartesianPoseImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianPoseImpedanceController::cartesianPoseCallback(
    const panda_ros::PoseCommand& msg) {
  elapsed_time_ = ros::Duration(0.0);
  for (auto i = 0; i < 16; i++) {
    desired_pose_[i] = msg.pose[i];
  }
  desired_movement_time_ = ros::Duration(msg.movement_time);
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

void CartesianPoseImpedanceController::impedanceModeCallback(
    const std_msgs::Int8& msg) {
  std::cout << "MODE " << mode << std::endl;
  if (mode == 2) {
    setDesiredStateToCurrent();
  }
  mode = msg.data;
}

void CartesianPoseImpedanceController::setDesiredStateToCurrent() {
  desired_movement_time_ = ros::Duration(5.0);
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  desired_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(panda_ros::CartesianPoseImpedanceController,
                       controller_interface::ControllerBase)
