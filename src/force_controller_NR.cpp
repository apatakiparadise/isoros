// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_panda_controller_swc/force_controller_NR.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/robot.h>
namespace franka_panda_controller_swc {

bool ForceControllerNR::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;

  sub_unity_publisher_ = node_handle.subscribe(
      "/unity_output", 20, &ForceControllerNR::Unity_publisher, this,
      ros::TransportHints().reliable().tcpNoDelay());

  ROS_WARN(
      "ForceControllerNR: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceControllerNR: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceControllerNR: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("ForceControllerNR: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceControllerNR: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceControllerNR: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceControllerNR: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceControllerNR: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceControllerNR: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceControllerNR: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  unity_publisher_.init(node_handle, "unity_input", 1);

  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_panda_controller_swc::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&ForceControllerNR::desiredMassParamCallback, this, _1, _2));

  unity_out_bool_.setZero();

  return true;
}

void ForceControllerNR::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();
  x_pre_ = robot_state.O_T_EE[12];
  y_pre_ = robot_state.O_T_EE[13];
  z_pre_ = robot_state.O_T_EE[14];
}

void ForceControllerNR::update(const ros::Time& /*time*/, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();

  //************* Free motion + Force field, Editor: Shih-Wen Chen *************
  Eigen::Matrix<double, 3,1> end_vel;
  Eigen::Matrix<double, 3,1> force_input;
  Eigen::Matrix<double, 3,3> force_const;  
  force_const.setZero();
  end_vel(0) = (robot_state.O_T_EE[12] - x_pre_) / period.toSec();
  end_vel(1) = (robot_state.O_T_EE[13] - y_pre_) / period.toSec();
  end_vel(2) = (robot_state.O_T_EE[14] - z_pre_) / period.toSec();
  
  force_const(0,1) = -F_K_;
  force_const(0,2) = F_K_;
  force_const(1,0) = F_K_;
  force_const(1,2) = -F_K_;
  force_const(2,0) = -F_K_;
  force_const(2,1) = F_K_;
  
  force_input = force_const * end_vel;
  // Get rid of "Nan"
  if (isnan(force_input(0))) {
    force_input(0) = 0.0;
  }
  if (isnan(force_input(1))) {
    force_input(1) = 0.0;
  }
  if (isnan(force_input(2))) {
    force_input(2) = 0.0;
  }
  /*
  desired_force_torque(0) = 30 * force_input(0) - 0.3 * force_input(0);
  desired_force_torque(1) = 50 * force_input(1) - 0.3 * force_input(1);
  desired_force_torque(2) = 50 * force_input(2) - 0.3 * force_input(2) + desired_mass_ * -9.81 + unity_out_bool_(1) * 9.81;
  */
  desired_force_torque(0) = 30 * force_input(0);
  desired_force_torque(1) = 40 * force_input(1);
  desired_force_torque(2) = 40 * force_input(2) + desired_mass_ * -9.81 + unity_out_bool_(1) * 9.81;
  
  //************* Free motion + force field, Editor: Shih-Wen Chen *************

  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_J_d);
  
  // Update the robot state for end-effector velocities
  x_pre_ = robot_state.O_T_EE[12];
  y_pre_ = robot_state.O_T_EE[13];
  z_pre_ = robot_state.O_T_EE[14];

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  if (rate_trigger_() && unity_publisher_.trylock()) {
    std::array<double, 16> uni_input_p = robot_state.O_T_EE;
    //std::array<double, 6> uni_input_f = robot_state.O_F_ext_hat_K;
    std::array<double, 6> uni_input_f = robot_state.K_F_ext_hat_K;
    unity_publisher_.msg_.linear.x = uni_input_p[12];
    unity_publisher_.msg_.linear.y = uni_input_p[13];
    unity_publisher_.msg_.linear.z = uni_input_p[14];
    unity_publisher_.msg_.angular.x = uni_input_f[0];
    unity_publisher_.msg_.angular.y = uni_input_f[1];
    unity_publisher_.msg_.angular.z = uni_input_f[2];
    unity_publisher_.unlockAndPublish();
  }
  /*
  std::cout << "  x: "<< robot_state.K_F_ext_hat_K[0];
  std::cout << "  y: "<< robot_state.K_F_ext_hat_K[1];
  std::cout << "  z: "<< robot_state.K_F_ext_hat_K[2] << "\n";*/

  // Update signals changed online through dynamic reconfigure
  desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
  k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
  k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

void ForceControllerNR::desiredMassParamCallback(
    franka_panda_controller_swc::desired_mass_paramConfig& config,
    uint32_t /*level*/) {
  target_mass_ = config.desired_mass;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
}

Eigen::Matrix<double, 7, 1> ForceControllerNR::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


//************* Unity subscriber, Editor: Shih-Wen Chen *************
void ForceControllerNR::Unity_publisher(
    const geometry_msgs::Vector3ConstPtr& msg) {
  unity_out_bool_ << msg->x, msg->y, msg->z;
}
//************* Unity subscriber, Editor: Shih-Wen Chen *************

}  // namespace franka_panda_controller_swc

PLUGINLIB_EXPORT_CLASS(franka_panda_controller_swc::ForceControllerNR,
                       controller_interface::ControllerBase)
