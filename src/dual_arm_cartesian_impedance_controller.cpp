// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_panda_controller_swc/dual_arm_cartesian_impedance_controller.h>

#include <cmath>
#include <functional>
#include <memory>

#include <controller_interface/controller_base.h>
#include <eigen_conversions/eigen_msg.h>
#include <franka/robot_state.h>
#include <franka_panda_controller_swc/pseudo_inversion.h>
#include <franka_hw/trigger_rate.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace franka_panda_controller_swc {

bool DualArmCartesianImpedanceController::initArm(
    hardware_interface::RobotHW* robot_hw,
    const std::string& arm_id,
    const std::vector<std::string>& joint_names) {
  FrankaDataContainer arm_data;
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    arm_data.state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      arm_data.joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DualArmCartesianImpedanceController: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }

  arm_data.position_d_.setZero();
  arm_data.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  arm_data.position_d_target_.setZero();
  arm_data.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  arm_data.cartesian_stiffness_.setZero();
  arm_data.cartesian_damping_.setZero();

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));

  return true;
}

bool DualArmCartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                                      ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  if (!node_handle.getParam("left/arm_id", left_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Could not read parameter left_arm_id_");
    return false;
  }
  std::vector<std::string> left_joint_names;
  if (!node_handle.getParam("left/joint_names", left_joint_names) || left_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmCartesianImpedanceController: Invalid or no left_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("right/arm_id", right_arm_id_)) {
    ROS_ERROR_STREAM(
        "DualArmCartesianImpedanceController: Could not read parameter right_arm_id_");
    return false;
  }

  boost::function<void(const geometry_msgs::PoseStamped::ConstPtr&)> callback =
      boost::bind(&DualArmCartesianImpedanceController::targetPoseCallback, this, _1);

  ros::SubscribeOptions subscribe_options;
  subscribe_options.init("centering_frame_target_pose", 1, callback);
  subscribe_options.transport_hints = ros::TransportHints().reliable().tcpNoDelay();
  sub_target_pose_left_ = node_handle.subscribe(subscribe_options);

  std::vector<std::string> right_joint_names;
  if (!node_handle.getParam("right/joint_names", right_joint_names) ||
      right_joint_names.size() != 7) {
    ROS_ERROR(
        "DualArmCartesianImpedanceController: Invalid or no right_joint_names parameters "
        "provided, "
        "aborting controller init!");
    return false;
  }

  bool left_success = initArm(robot_hw, left_arm_id_, left_joint_names);
  bool right_success = initArm(robot_hw, right_arm_id_, right_joint_names);

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<
      franka_panda_combined_controller_swc::dual_arm_compliance_paramConfig>>(
      dynamic_reconfigure_compliance_param_node_);

  dynamic_server_compliance_param_->setCallback(boost::bind(
      &DualArmCartesianImpedanceController::complianceParamCallback, this, _1, _2));

  // Get the transformation from right_O_frame to left_O_frame
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try {
    if (listener.waitForTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                                  ros::Duration(4.0))) {
      listener.lookupTransform(left_arm_id_ + "_link0", right_arm_id_ + "_link0", ros::Time(0),
                               transform);
    } else {
      ROS_ERROR(
          "DualArmCartesianImpedanceController: Failed to read transform from %s to %s. "
          "Aborting init!",
          (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str());
      return false;
    }
  } catch (tf::TransformException& ex) {
    ROS_ERROR("DualArmCartesianImpedanceController: %s", ex.what());
    return false;
  }
  tf::transformTFToEigen(transform, Ol_T_Or_);  // NOLINT (readability-identifier-naming)

  // Setup publisher for the centering frame.
  publish_rate_ = franka_hw::TriggerRate(30.0);
  center_frame_pub_.init(node_handle, "centering_frame", 1, true);
  vel_publisher_.init(node_handle, "velocity", 1);

  return left_success && right_success;
}

void DualArmCartesianImpedanceController::startingArm(FrankaDataContainer& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = arm_data.state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  arm_data.position_d_ = initial_transform.translation();
  arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  arm_data.position_d_target_ = initial_transform.translation();
  arm_data.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  arm_data.q_d_nullspace_ = q_initial;
  // last joint of null space impedance controller
  arm_data.q_d_nullspace_[6] = -0.761;
}

void DualArmCartesianImpedanceController::starting(const ros::Time& /*time*/) {
  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
  }
  franka::RobotState robot_state_right =
      arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
  Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  Eigen::Affine3d Or_T_EEr(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
  EEr_T_EEl_ =
      Or_T_EEr.inverse() * Ol_T_Or_.inverse() * Ol_T_EEl;  // NOLINT (readability-identifier-naming)
  EEl_T_C_.setIdentity();
  Eigen::Vector3d EEr_r_EEr_EEl =  // NOLINT (readability-identifier-naming)
      EEr_T_EEl_.translation();    // NOLINT (readability-identifier-naming)
  EEl_T_C_.translation() = -0.5 * EEr_T_EEl_.inverse().rotation() * EEr_r_EEr_EEl;
}

void DualArmCartesianImpedanceController::update(const ros::Time& /*time*/,
                                                        const ros::Duration& /*period*/) {
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second);
  }
  if (publish_rate_()) {
    publishCenteringPose();
  }
}

void DualArmCartesianImpedanceController::updateArm(FrankaDataContainer& arm_data) {
  // get state variables
  franka::RobotState robot_state = arm_data.state_handle_->getRobotState();
  std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  
  //*******************Position: start*******************
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  auto& right_arm_data = arms_data_.at(right_arm_id_);
  if (robot_state.O_T_EE[14] >= 0.35) {
    if (robot_state.O_T_EE[12] >= 0.15) {
      arm_data.position_d_target_up = arm_data.position_d_target_;
    } else if (robot_state.O_T_EE[12] > -0.10 && robot_state.O_T_EE[12] < 0.15){
      arm_data.position_d_target_up = arm_data.position_d_target_;
      right_arm_data.position_d_target_up(0) = 0.0;
      right_arm_data.position_d_target_up(1) = -0.15;
      left_arm_data.position_d_target_up(0) = 0.0;
      left_arm_data.position_d_target_up(1) = 0.15;
    }else if (robot_state.O_T_EE[12] < -0.10 && robot_state.q[0] < 0) {
      arm_data.position_d_target_up = arm_data.position_d_target_;
      arm_data.position_d_target_up(0) = -arm_data.position_d_target_(0);
    } 
  } else {
    if (robot_state.O_T_EE[12] >= 0.15) {
      arm_data.position_d_target_up = arm_data.position_d_target_;
      arm_data.position_d_target_up(0) = 0.35;
      arm_data.position_d_target_up(2) = 0.28;
    } else {
      right_arm_data.position_d_target_up(0) = 0.1;
      right_arm_data.position_d_target_up(1) = -0.25;
      right_arm_data.position_d_target_up(2) = 0.28;
      left_arm_data.position_d_target_up(0) = 0.1;
      left_arm_data.position_d_target_up(1) = 0.25;
      left_arm_data.position_d_target_up(2) = 0.28;
    }
  }
  //*******************Position: end*******************
  
  
  //*******************Orientation: start*******************
  std::array<double, 16> rotational_hand_l;
  rotational_hand_l = {0.0, 1.0, 0.0, 0.0,
                       1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, -1.0, 0.0,
                       0.288125, 0.202355, 0.692674, 1.0};
  Eigen::Affine3d left_transform(Eigen::Matrix4d::Map(rotational_hand_l.data()));
  left_arm_data.orientation_d_target_ = Eigen::Quaterniond(left_transform.linear());

  std::array<double, 16> rotational_hand_r;
  rotational_hand_r = {0.0, 1.0, 0.0, 0.0,
                       1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, -1.0, 0.0,
                       0.288125, 0.202355, 0.692674, 1.0};
  Eigen::Affine3d right_transform(Eigen::Matrix4d::Map(rotational_hand_r.data()));
  right_arm_data.orientation_d_target_ = Eigen::Quaterniond(right_transform.linear());

  /*
  std::array<double, 16> rotational_hand_l;
  rotational_hand_l = {1.0, 0.0, 0.0, 0.0,
                       0.0, -0.707, -0.707, 0.0,
                       0.0, 0.707, -0.707, 0.0,
                       0.288125, 0.202355, 0.692674, 1.0};
  Eigen::Affine3d left_transform(Eigen::Matrix4d::Map(rotational_hand_l.data()));
  left_arm_data.orientation_d_target_ = Eigen::Quaterniond(left_transform.linear());

  std::array<double, 16> rotational_hand_r;
  rotational_hand_r = {1.0, 0.0, 0.0, 0.0,
                       0.0, -0.707, 0.707, 0.0,
                       0.0, -0.707, -0.707, 0.0,
                       0.288125, 0.202355, 0.692674, 1.0};
  Eigen::Affine3d right_transform(Eigen::Matrix4d::Map(rotational_hand_r.data()));
  right_arm_data.orientation_d_target_ = Eigen::Quaterniond(right_transform.linear());
  */
  //*******************Orientation: end*******************
 

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - arm_data.position_d_;

  // orientation error
  if (arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * arm_data.orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_panda_controller_swc::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-arm_data.cartesian_stiffness_ * error -
                                      arm_data.cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (arm_data.nullspace_stiffness_ * (arm_data.q_d_nullspace_ - q) -
                        (2.0 * sqrt(arm_data.nullspace_stiffness_)) * dq);
  

  /*
  //*******************Mass: start*******************
  Eigen::VectorXd tau_f(7), desired_force_torque(6);
  desired_force_torque.setZero();
  desired_force_torque(2) = 1 * -9.81;
  tau_f << jacobian.transpose() * desired_force_torque;
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + tau_f;
  //*******************Mass: end*******************
  */
  
  
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(arm_data, tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    arm_data.joint_handles_[i].setCommand(tau_d(i));
  }

  
  //*******************joint limit: start*******************
  if (robot_state.q[0] > 2.5 || robot_state.q[0] < -2.5 ||
      robot_state.q[1] > 1.5 || 
      robot_state.q[2] > 2.5 || robot_state.q[2] < -2.5 ||
      robot_state.q[3] > -0.3 || 
      robot_state.q[4] > 2.5 || robot_state.q[4] < -2.5 ||
      robot_state.q[5] > 3.5 || robot_state.q[5] < 0.3 ||
      robot_state.q[6] > 2.7 || robot_state.q[6] < -2.7) {
    arm_data.nullspace_stiffness_target_ = 5.0;
  } else if (robot_state.q[1] < -1.5 || robot_state.q[3] < -2.9) {
    if ((robot_state.q[0] > -0.5 && robot_state.q[0] < 0.5) &&
        (robot_state.q[2] > -0.5 && robot_state.q[2] < 0.5)) {
          arm_data.nullspace_stiffness_target_ = 8.0;
    } else {
      arm_data.nullspace_stiffness_target_ = 5.0;
    }
  } else {
    arm_data.nullspace_stiffness_target_ = 0.0;
  } 
  //*******************joint limit: end*******************


  /*
  //*******************Speed limit: start*******************
  Eigen::VectorXd end_vel(6), end_vel_trans(3);
  end_vel = jacobian * dq;
  for (size_t i = 0; i < 3; ++i) {
    end_vel_trans(i) = end_vel(i);
  }
  
  //*******************Speed limit: end*******************
  */

  //*******************Collision avoidance: start*******************
  franka::RobotState robot_state_right = arms_data_.at(right_arm_id_).state_handle_->getRobotState();
  franka::RobotState robot_state_left = arms_data_.at(left_arm_id_).state_handle_->getRobotState();
  Eigen::VectorXd r_position_c(3), l_position_c(3), r_position_elbow(3), l_position_elbow(3);
  Eigen::VectorXd r_position_b(3), l_position_b(3), r_position_w(3), l_position_w(3), r_position_s(3), l_position_s(3);
  Eigen::VectorXd r_joint_cos(7), r_joint_sin(7), l_joint_cos(7), l_joint_sin(7);
  double distance_robots = 0.8;
  double flag = 0;
  double logistic_fun = 0;
  double min_dis = 20;

  //**************end effector**************

  // end effector
  for (size_t i = 0; i < 3; i++) {
    r_position_c(i) = robot_state_right.O_T_EE[12+i];
    l_position_c(i) = robot_state_left.O_T_EE[12+i];
  }
  r_position_c(1) = r_position_c(1) - 0.06;
  l_position_c(1) = l_position_c(1) - distance_robots + 0.06;
  r_position_c(2) = r_position_c(2) + 0.08;
  l_position_c(2) = l_position_c(2) + 0.08;
  if ((r_position_c - l_position_c).norm() >= 0.2 && (r_position_c - l_position_c).norm() < 0.21) {
    logistic_fun = 0;
    arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  } else if ((r_position_c - l_position_c).norm() < 0.2){
    logistic_fun = 50/(1+exp(-100*(0.18-(r_position_c - l_position_c).norm())));
    arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
    flag = 1;
  }
  // std::cout << logistic_fun << "  " << (0.13-(r_position_c - l_position_c).norm()) << "\n";
  //**************base and shoulder**************

  if (flag != 1) {
  // base
  r_position_b.setZero();
  l_position_b.setZero();
  r_position_b(2) = 0.075;
  l_position_b(2) = 0.075;
  r_position_b(1) = -0.05;
  l_position_b(1) = - distance_robots + 0.05;
  // shoulder
  r_position_s = r_position_b;
  l_position_s = l_position_b;
  r_position_s(2) = 0.333;
  l_position_s(2) = 0.333;

  if ((r_position_b - l_position_c).norm() < 0.2 || (r_position_s - l_position_c).norm() < 0.2) {
    min_dis = fmin((r_position_b - l_position_c).norm(), (r_position_s - l_position_c).norm());
    logistic_fun = 50/(1+exp(-100*(0.18-min_dis)));
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
    flag = 2;  
  }else if (((r_position_b - l_position_c).norm() >= 0.2 && (r_position_b - l_position_c).norm() < 0.21) || 
     ((r_position_s - l_position_c).norm() >= 0.2 && (r_position_s - l_position_c).norm() < 0.21)) {
    logistic_fun = 0;
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  }
  if ((l_position_b - r_position_c).norm() < 0.2 || (l_position_s - r_position_c).norm() < 0.2) {
    min_dis = fmin((l_position_b - r_position_c).norm(), (l_position_s - r_position_c).norm());
    logistic_fun = 50/(1+exp(-100*(0.18-min_dis)));
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
    flag = 2; 
  }else if (((l_position_b - r_position_c).norm() >= 0.2 && (l_position_b - r_position_c).norm() < 0.21) || 
     ((l_position_s - r_position_c).norm() >= 0.2 && (l_position_s - r_position_c).norm() < 0.21)) {
    logistic_fun = 0;
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  }
  }

  //**************elbow and wrist**************

  if (flag != 1 && flag != 2) {
  // lenght of links
  double L1 = 0.333;
  double L2 = 0.316;
  double L3 = 0.0825;
  double L4 = 0.29;  // till franka icon
  for (size_t i = 0; i < 7; i++) {
    r_joint_cos(i) = std::cos(robot_state_right.q[i]);
    r_joint_sin(i) = std::sin(robot_state_right.q[i]);
    l_joint_cos(i) = std::cos(robot_state_left.q[i]);
    l_joint_sin(i) = std::sin(robot_state_left.q[i]);
  }
   // elbow 
  r_position_elbow(0) = L2 * r_joint_cos(0) * r_joint_sin(1) - L3 * (r_joint_sin(0) * r_joint_sin(2) - r_joint_cos(0) * r_joint_cos(1) * r_joint_cos(2));
  r_position_elbow(1) = L2 * r_joint_sin(0) * r_joint_sin(1) + L3 * (r_joint_cos(0) * r_joint_sin(2) + r_joint_sin(0) * r_joint_cos(1) * r_joint_cos(2));
  r_position_elbow(2) = L1 + L2 * r_joint_cos(1) - L3 * r_joint_cos(2) * r_joint_sin(1);
  l_position_elbow(0) = L2 * l_joint_cos(0) * l_joint_sin(1) - L3 * (l_joint_sin(0) * l_joint_sin(2) - l_joint_cos(0) * l_joint_cos(1) * l_joint_cos(2));
  l_position_elbow(1) = L2 * l_joint_sin(0) * l_joint_sin(1) + L3 * (l_joint_cos(0) * l_joint_sin(2) + l_joint_sin(0) * l_joint_cos(1) * l_joint_cos(2));
  l_position_elbow(2) = L1 + L2 * l_joint_cos(1) - L3 * l_joint_cos(2) * l_joint_sin(1);
  l_position_elbow(1) = l_position_elbow(1) - distance_robots ;
  // wrist
  r_position_w(0) = r_position_elbow(0) + 
                    L3 * (r_joint_cos(3) * (r_joint_sin(0) * r_joint_sin(2) - r_joint_cos(0) * r_joint_cos(1) * r_joint_cos(2)) - r_joint_cos(0) * r_joint_sin(1) * r_joint_sin(3)) + 
                    L4 * (r_joint_sin(3) * (r_joint_sin(0) * r_joint_sin(2) - r_joint_cos(0) * r_joint_cos(1) * r_joint_cos(2)) + r_joint_cos(0) * r_joint_sin(1) * r_joint_cos(3));
  r_position_w(1) = r_position_elbow(1) - 
                    L3 * (r_joint_cos(3) * (r_joint_cos(0) * r_joint_sin(2) + r_joint_sin(0) * r_joint_cos(1) * r_joint_cos(2)) + r_joint_sin(0) * r_joint_sin(1) * r_joint_sin(3)) - 
                    L4 * (r_joint_sin(3) * (r_joint_cos(0) * r_joint_sin(2) + r_joint_sin(0) * r_joint_cos(1) * r_joint_cos(2)) - r_joint_sin(0) * r_joint_sin(1) * r_joint_cos(3));
  r_position_w(2) = r_position_elbow(2) - L3 * ((r_joint_cos(1) * r_joint_sin(3)) - r_joint_cos(2) * r_joint_cos(3) * r_joint_sin(1)) + 
                    L4 * (r_joint_cos(1) * r_joint_cos(3) + r_joint_cos(2) * r_joint_sin(1) * r_joint_sin(3));
  l_position_w(0) = l_position_elbow(0) + 
                    L3 * (l_joint_cos(3) * (l_joint_sin(0) * l_joint_sin(2) - l_joint_cos(0) * l_joint_cos(1) * l_joint_cos(2)) - l_joint_cos(0) * l_joint_sin(1) * l_joint_sin(3)) + 
                    L4 * (l_joint_sin(3) * (l_joint_sin(0) * l_joint_sin(2) - l_joint_cos(0) * l_joint_cos(1) * l_joint_cos(2)) + l_joint_cos(0) * l_joint_sin(1) * l_joint_cos(3));
  l_position_w(1) = l_position_elbow(1) - 
                    L3 * (l_joint_cos(3) * (l_joint_cos(0) * l_joint_sin(2) + l_joint_sin(0) * l_joint_cos(1) * l_joint_cos(2)) + l_joint_sin(0) * l_joint_sin(1) * l_joint_sin(3)) - 
                    L4 * (l_joint_sin(3) * (l_joint_cos(0) * l_joint_sin(2) + l_joint_sin(0) * l_joint_cos(1) * l_joint_cos(2)) - l_joint_sin(0) * l_joint_sin(1) * l_joint_cos(3));
  l_position_w(2) = l_position_elbow(2) - L3 * ((l_joint_cos(1) * l_joint_sin(3)) - l_joint_cos(2) * l_joint_cos(3) * l_joint_sin(1)) + 
                    L4 * (l_joint_cos(1) * l_joint_cos(3) + l_joint_cos(2) * l_joint_sin(1) * l_joint_sin(3));
  
  // left wrist v.s. right end effector
  if ((r_position_w - l_position_c).norm() >= 0.25 && (r_position_w - l_position_c).norm() < 0.26) {
    logistic_fun = 0;
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  } else if ((r_position_w - l_position_c).norm() < 0.25) {
    logistic_fun = 50/(1+exp(-100*(0.23-(r_position_w - l_position_c).norm())));
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
  }
  // left wrist v.s. right end effector
  if ((l_position_w - r_position_c).norm() >= 0.25 && (l_position_w - r_position_c).norm() < 0.26) {
    logistic_fun = 0;
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  } else if ((l_position_w - r_position_c).norm() < 0.25) {
    logistic_fun = 50/(1+exp(-100*(0.23-(l_position_w - r_position_c).norm())));
    left_arm_data.nullspace_stiffness_target_ = 5.0;
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
  }
 
  // right elbow v.s. left end effector
  if ((r_position_elbow - l_position_c).norm() >= 0.23 && (r_position_elbow - l_position_c).norm() < 0.24) {
    logistic_fun = 0;
    right_arm_data.nullspace_stiffness_target_ = 0.0;
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  }else if ((r_position_elbow - l_position_c).norm() < 0.23) {
    min_dis = fmin((r_position_elbow - l_position_c).norm(), (r_position_w - l_position_c).norm());
    logistic_fun = 50/(1+exp(-100*(0.21-min_dis)));
    right_arm_data.nullspace_stiffness_target_ = 5.0;
    left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
  }
  // left elbow v.s. right end effector
  if ((l_position_elbow - r_position_c).norm() >= 0.23 && (l_position_elbow - r_position_c).norm() < 0.24) {
    logistic_fun = 0;
    left_arm_data.nullspace_stiffness_target_ = 0.0;
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  } else if ((l_position_elbow - r_position_c).norm() < 0.23) {
    min_dis = fmin((l_position_elbow - r_position_c).norm(), (l_position_w - r_position_c).norm());
    logistic_fun = 50/(1+exp(-100*(0.21-min_dis)));
    left_arm_data.nullspace_stiffness_target_ = 5.0;
    right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
  }
  // left elbow v.s. right elbow
  if ((l_position_elbow - r_position_elbow).norm() >= 0.2 && (l_position_elbow - r_position_elbow).norm() < 0.21) {
    left_arm_data.nullspace_stiffness_target_ = 0.0;
    right_arm_data.nullspace_stiffness_target_ = 0.0;
  } else if ((l_position_elbow - r_position_elbow).norm() < 0.2) {
    left_arm_data.nullspace_stiffness_target_ = 5.0;
    right_arm_data.nullspace_stiffness_target_ = 5.0;
  } 
  // left wrist v.s. right wrist
  if ((l_position_w - r_position_w).norm() >= 0.34 && (l_position_w - r_position_w).norm() < 0.35) {
    logistic_fun = 0;
    arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
  } else if ((l_position_w - r_position_w).norm() < 0.34) {
    logistic_fun = 50/(1+exp(-200*(0.32-(l_position_w - r_position_w).norm())));
    arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << logistic_fun * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(logistic_fun) * Eigen::Matrix3d::Identity();
    flag = 4;
  }
  }
  //*******************Collision avoidance: end*******************
  if (robot_state.O_T_EE[14] < 0.35 && robot_state.O_T_EE[12] < 0.15) {
    if (r_position_c(0) > l_position_c(0)) {
      left_arm_data.position_d_target_up(0) = -0.15;
    } else {
      right_arm_data.position_d_target_up(0) = -0.15;
    }
  }
  if (flag == 4) {
    if (r_position_c(0) > l_position_c(0)) {
      right_arm_data.position_d_target_up(2) = 0.55;
      left_arm_data.position_d_target_up(2) = 0.20;
    } else {
      left_arm_data.position_d_target_up(2) = 0.55;
      right_arm_data.position_d_target_up(2) = 0.20;
    }
  }

  //*******************Speed : start*******************
  double vel_r;
  double vel_l;
  double max_r, max_l;
  double min_r, min_l;
  Eigen::VectorXd end_vel_r(6), end_vel_r_linear(3), end_vel_l(6), end_vel_l_linear(3);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_r(robot_state_right.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_l(robot_state_left.dq.data());
  end_vel_r = jacobian * dq_r;
  end_vel_l = jacobian * dq_l;
  for (size_t i = 0; i < 3; ++i) {
    end_vel_r_linear(i) = end_vel_r(i);
    end_vel_l_linear(i) = end_vel_l(i);
  }
  //*******************Speed : end*******************
  if (vel_publisher_.trylock()) {
    min_r = fmin(fmin(fmin(fmin(fmin(end_vel_r_linear.norm(),pre_v1_r), pre_v2_r), pre_v3_r), pre_v4_r),pre_v5_r);
    min_l = fmin(fmin(fmin(fmin(fmin(end_vel_l_linear.norm(),pre_v1_l), pre_v2_l), pre_v3_l), pre_v4_l),pre_v5_l);
    max_r = fmax(fmax(fmax(fmax(fmax(end_vel_r_linear.norm(),pre_v1_r), pre_v2_r), pre_v3_r), pre_v4_r),pre_v5_r);
    max_l = fmax(fmax(fmax(fmax(fmax(end_vel_l_linear.norm(),pre_v1_l), pre_v2_l), pre_v3_l), pre_v4_l),pre_v5_l);
    vel_r = (end_vel_r_linear.norm() + pre_v1_r + pre_v2_r + pre_v3_r + pre_v4_r + pre_v5_r - min_r - max_r)/4;
    vel_l = (end_vel_l_linear.norm() + pre_v1_l + pre_v2_l + pre_v3_l + pre_v4_l + pre_v5_l - min_l - max_l)/4;
    vel_publisher_.msg_.right_vel = vel_r;
    vel_publisher_.msg_.left_vel = vel_l;
    vel_publisher_.msg_.distance = (r_position_c - l_position_c).norm();
    vel_publisher_.unlockAndPublish();
    pre_v1_r = pre_v2_r;
    pre_v2_r = pre_v3_r;
    pre_v3_r = pre_v4_r;
    pre_v4_r = pre_v5_r;
    pre_v5_r = pre_v6_r;
    pre_v6_r = end_vel_r_linear.norm();
    pre_v1_l = pre_v2_l;
    pre_v2_l = pre_v3_l;
    pre_v3_l = pre_v4_l;
    pre_v4_l = pre_v5_l;
    pre_v5_l = pre_v6_l;
    pre_v6_l = end_vel_l_linear.norm();

  }






  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  if (arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3) == 0 * Eigen::Matrix3d::Identity() ||
     arm_data.cartesian_damping_target_.topLeftCorner(3, 3) == 0 * Eigen::Matrix3d::Identity() || 
     arm_data.nullspace_stiffness_target_ == 0){
    arm_data.cartesian_stiffness_ = arm_data.filter_params_* 3 * arm_data.cartesian_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_ * 3) * arm_data.cartesian_stiffness_;
    arm_data.cartesian_damping_ = arm_data.filter_params_ * 3 * arm_data.cartesian_damping_target_ +
                                  (1.0 - arm_data.filter_params_ * 3) * arm_data.cartesian_damping_;
    arm_data.nullspace_stiffness_ = arm_data.filter_params_ * 3 * arm_data.nullspace_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_ * 3) * arm_data.nullspace_stiffness_;
  } else {
    arm_data.cartesian_stiffness_ = arm_data.filter_params_ * arm_data.cartesian_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_ ) * arm_data.cartesian_stiffness_;
    arm_data.cartesian_damping_ = arm_data.filter_params_  * arm_data.cartesian_damping_target_ +
                                  (1.0 - arm_data.filter_params_) * arm_data.cartesian_damping_;
    arm_data.nullspace_stiffness_ = arm_data.filter_params_ * arm_data.nullspace_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_) * arm_data.nullspace_stiffness_;
  }

  arm_data.position_d_ = arm_data.filter_params_ *0.5 * arm_data.position_d_target_up +
                         (1.0 - arm_data.filter_params_ *0.5) * arm_data.position_d_;
  arm_data.orientation_d_ =
      arm_data.orientation_d_.slerp(arm_data.filter_params_ * 0.5, arm_data.orientation_d_target_);

}

Eigen::Matrix<double, 7, 1> DualArmCartesianImpedanceController::saturateTorqueRate(
    const FrankaDataContainer& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}

void DualArmCartesianImpedanceController::complianceParamCallback(
    franka_panda_combined_controller_swc::dual_arm_compliance_paramConfig& config,
    uint32_t /*level*/) {
  auto& left_arm_data = arms_data_.at(left_arm_id_);
  left_arm_data.cartesian_stiffness_target_.setIdentity();
  left_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.left_translational_stiffness * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.left_rotational_stiffness * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_damping_target_.setIdentity();

  left_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(config.left_translational_stiffness) * Eigen::Matrix3d::Identity();
  left_arm_data.cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2 * sqrt(config.left_rotational_stiffness) * Eigen::Matrix3d::Identity();
  left_arm_data.nullspace_stiffness_target_ = config.left_nullspace_stiffness;

  auto& right_arm_data = arms_data_.at(right_arm_id_);
  right_arm_data.cartesian_stiffness_target_.setIdentity();
  right_arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.right_translational_stiffness * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.right_rotational_stiffness * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_damping_target_.setIdentity();

  right_arm_data.cartesian_damping_target_.topLeftCorner(3, 3)
      << 2 * sqrt(config.right_translational_stiffness) * Eigen::Matrix3d::Identity();
  right_arm_data.cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2 * sqrt(config.right_rotational_stiffness) * Eigen::Matrix3d::Identity();
  right_arm_data.nullspace_stiffness_target_ = config.right_nullspace_stiffness;
}

void DualArmCartesianImpedanceController::targetPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  try {
    if (msg->header.frame_id != left_arm_id_ + "_link0") {
      ROS_ERROR_STREAM(
          "DualArmCartesianImpedanceController: Got pose target with invalid"
          " frame_id "
          << msg->header.frame_id << ". Expected " << left_arm_id_ + "_link0");
      return;
    }

    // Set target for the left robot.
    auto& left_arm_data = arms_data_.at(left_arm_id_);
    Eigen::Affine3d Ol_T_C;  // NOLINT (readability-identifier-naming)
    tf::poseMsgToEigen(msg->pose, Ol_T_C);
    Eigen::Affine3d Ol_T_EEl_d =      // NOLINT (readability-identifier-naming)
        Ol_T_C * EEl_T_C_.inverse();  // NOLINT (readability-identifier-naming)
    left_arm_data.position_d_target_ = Ol_T_EEl_d.translation();
    Eigen::Quaterniond last_orientation_d_target(left_arm_data.orientation_d_target_);
    Eigen::Quaterniond new_orientation_target(Ol_T_EEl_d.linear());
    if (last_orientation_d_target.coeffs().dot(new_orientation_target.coeffs()) < 0.0) {
      new_orientation_target.coeffs() << -new_orientation_target.coeffs();
    }
    Ol_T_EEl_d.linear() = new_orientation_target.matrix();
    left_arm_data.orientation_d_target_ = Ol_T_EEl_d.linear();

    // Compute target for the right endeffector given the static desired transform from left to
    // right endeffector.
    Eigen::Affine3d Or_T_EEr_d = Ol_T_Or_.inverse()     // NOLINT (readability-identifier-naming)
                                 * Ol_T_EEl_d *         // NOLINT (readability-identifier-naming)
                                 EEr_T_EEl_.inverse();  // NOLINT (readability-identifier-naming)
    auto& right_arm_data = arms_data_.at(right_arm_id_);
    right_arm_data.position_d_target_ =
        Or_T_EEr_d.translation();  // NOLINT (readability-identifier-naming)
    last_orientation_d_target = Eigen::Quaterniond(right_arm_data.orientation_d_target_);
    right_arm_data.orientation_d_target_ =
        Or_T_EEr_d.linear();  // NOLINT (readability-identifier-naming)
    if (last_orientation_d_target.coeffs().dot(right_arm_data.orientation_d_target_.coeffs()) <
        0.0) {
      right_arm_data.orientation_d_target_.coeffs()
          << -right_arm_data.orientation_d_target_.coeffs();
    }

  } catch (std::out_of_range& ex) {
    ROS_ERROR_STREAM("DualArmCartesianImpedanceController: Exception setting target poses.");
  }
}

void DualArmCartesianImpedanceController::publishCenteringPose() {
  if (center_frame_pub_.trylock()) {
    franka::RobotState robot_state_left =
        arms_data_.at(left_arm_id_).state_handle_->getRobotState();
    Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
        robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
    Eigen::Affine3d Ol_T_C = Ol_T_EEl * EEl_T_C_;   // NOLINT (readability-identifier-naming)
    tf::poseEigenToMsg(Ol_T_C, center_frame_pub_.msg_.pose);
    center_frame_pub_.msg_.header.frame_id = left_arm_id_ + "_link0";
    center_frame_pub_.unlockAndPublish();
  }
}

}  // namespace franka_panda_controller_swc

PLUGINLIB_EXPORT_CLASS(franka_panda_controller_swc::DualArmCartesianImpedanceController,
                       controller_interface::ControllerBase)
