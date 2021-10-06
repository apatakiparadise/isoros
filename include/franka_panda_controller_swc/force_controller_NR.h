// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>
#include <franka_panda_controller_swc/UnityInput.h>
#include <franka_panda_controller_swc/desired_mass_paramConfig.h>
#include <geometry_msgs/Vector3.h>

namespace franka_panda_controller_swc {

class ForceControllerNR : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  double desired_mass_{0.0};
  double target_mass_{0.0};
  double k_p_{0.0};
  double k_i_{0.0};

  double F_K_{1.0};
  double x_pre_{0.0};
  double y_pre_{0.0};
  double z_pre_{0.0};
  Eigen::Vector3d unity_out_bool_;

  double target_k_p_{0.0};
  double target_k_i_{0.0};
  double filter_gain_{0.001};
  Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 7, 1> tau_error_;
  static constexpr double kDeltaTauMax{1.0};

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_panda_controller_swc::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_panda_controller_swc::desired_mass_paramConfig& config,
                                uint32_t level);
  // Subscribe from Unity
  ros::Subscriber sub_unity_publisher_;
  void Unity_publisher(const geometry_msgs::Vector3ConstPtr& msg);

  // Publisher to Unity
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<UnityInput> unity_publisher_;
};

}  // namespace franka_panda_controller_swc
