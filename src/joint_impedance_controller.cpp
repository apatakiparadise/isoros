#include <franka_panda_controller_swc/joint_impedance_controller.h>
#include <cmath>
#include <memory>
#include <iostream>
#include <array>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <franka_panda_controller_swc/pseudo_inversion.h>
#include <franka/robot_state.h>

namespace franka_panda_controller_swc {

bool JointImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "JointImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Exception getting cartesian pose handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

////////// Trajection Planning <Start> --- editor: Shih-Wen Chen //////////

void JointImpedanceController::starting(const ros::Time& /*time*/) {
  // initial end-effector config
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  // initial eblow config
  initial_elbow_ = cartesian_pose_handle_->getRobotState().elbow_d;

  // final end-effector config

  // these configs do not need elbow
  /*end_pose_ = {1.0, 0.0, 0.0, 0.0,
                0.0, -1.0, 0.0, 0.0,
                0.0, 0.0, -1.0, 0.0,
                -0.072078, 0.537113, 0.547998, 1.0};*/
  /*end_pose_ = {0.0, 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                -0.328405, 0.549474, 0.785429, 1.0};*/
  end_pose_ = {0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                1.0, 0.0, 0.0, 0.0,
                0.012693, 0.536757, 0.410588, 1.0};

  // this config needs elbow (failed)
  /*end_pose_ = {0.0, 1.0, 0.0, 0.0,
              -1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0,
              -0.328405, 0.549474, 0.785429, 1.0};*/

  // final end-effector config
  end_elbow_ = {1.5, -1.0};
}

void JointImpedanceController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  // accummulating time
  accu_time_ += period.toSec();
  
  // convert std to Eigen format
  Eigen::Map<Eigen::Matrix<double, 4, 4>> init_com(initial_pose_.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> end_com(end_pose_.data());
  Eigen::Matrix<double, 4, 4> pose_desired;
  Eigen::Matrix<double, 4, 4> pervision_pose;

  // screw motion variation
  double T = 10;
  double a_3 = 10/pow(T,3);
  double a_4 = -15/pow(T,4);
  double a_5 = 6/pow(T,5);
  double s = a_3 * pow(accu_time_,3) + a_4 * pow(accu_time_,4) + a_5 * pow(accu_time_,5);
  
  // std array for input command
  std::array<double, 16> command_mat_pose = initial_pose_;
  std::array<double, 2> command_mat_elbow = initial_elbow_;
  
  if (accu_time_ <= T) {
    // from start config to end config
    // pose traj (screw motion) -> X_start*(exp(log(inv(X_start)*X_end)s))
    Eigen::Matrix<double, 4, 4> initial_pose_inv_ = init_com.inverse();
    inv_init_end_ = initial_pose_inv_ * end_com;
    log_init_end_s_ = inv_init_end_.log() * s;
    pose_desired = init_com * log_init_end_s_.exp();

    // elbow traj
    command_mat_elbow[0] = initial_elbow_[0] + s * (end_elbow_[0] - initial_elbow_[0]);

  } else if (accu_time_ <= (T+5)){
    // stop 5 seconds
    pose_desired = end_com;
    command_mat_elbow = end_elbow_;
    
  } else {
    // sin-motion
    if (vel_current_ <= vel_max_) {
      vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
    }
    vel_current_ = std::fmin(vel_current_, vel_max_);
    angle_ += period.toSec() *0.5* vel_current_ / std::fabs(radius_);
    if (angle_ > 2 * M_PI) {
      angle_ -= 2 * M_PI;
    }
    pose_desired = end_com;
    command_mat_elbow = end_elbow_;

  }

  
  // transfer Eigne matrix format to std vector
  command_mat_pose[0] = pose_desired.data()[0];
  command_mat_pose[1] = pose_desired.data()[1];
  command_mat_pose[2] = pose_desired.data()[2];
  command_mat_pose[4] = pose_desired.data()[4];
  command_mat_pose[5] = pose_desired.data()[5];
  command_mat_pose[6] = pose_desired.data()[6];
  command_mat_pose[8] = pose_desired.data()[8];
  command_mat_pose[9] = pose_desired.data()[9];
  command_mat_pose[10] = pose_desired.data()[10];
  command_mat_pose[12] = pose_desired.data()[12];
  command_mat_pose[13] = pose_desired.data()[13];
  command_mat_pose[14] = pose_desired.data()[14];
  
  
  // input the command to robot
  if (accu_time_ <= T+5){
    cartesian_pose_handle_->setCommand(command_mat_pose);
  //  cartesian_pose_handle_->setCommand(command_mat_pose, command_mat_elbow);
  } else {
    command_mat_pose[14] += radius_ * std::sin(angle_);
    cartesian_pose_handle_->setCommand(command_mat_pose);
    command_mat_elbow[0] -= radius_ *2* std::sin(angle_);  
  //  cartesian_pose_handle_->setCommand(command_mat_pose, command_mat_elbow);
  }
  

  ////////// Trajection Planning <End> --- editor: Shih-Wen Chen //////////


  franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  /*
  std::cout << "  x: "<< robot_state.O_T_EE[12];
  std::cout << "  y: "<< robot_state.O_T_EE[13] << "\n";
  std::cout << "  z: "<< robot_state.O_T_EE[14] << "\n";*/
  

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> JointImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_panda_controller_swc

PLUGINLIB_EXPORT_CLASS(franka_panda_controller_swc::JointImpedanceController,
                       controller_interface::ControllerBase)
