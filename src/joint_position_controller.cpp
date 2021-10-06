// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_panda_controller_swc/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_panda_controller_swc {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }

  end_pose_ = {0.0, -0.819, 1.643, -1.607, -2.271, 1.439, 0.672};
  //end_pose_ = {0, -M_PI_4, 0, -3 * M_PI_4, M_PI_4, M_PI_2, M_PI_4};
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  std::array<double, 7> command_mat_joint = initial_pose_;

  double T = 10.0;
  if (elapsed_time_.toSec() <= T){
    for (size_t i = 0; i < 7; ++i) {
      command_mat_joint[i] = initial_pose_[i] - (10 * initial_pose_[i] - 10 * end_pose_[i]) * pow(elapsed_time_.toSec(),3) / pow(T,3) + 
                            (15 * initial_pose_[i] - 15 * end_pose_[i]) * pow(elapsed_time_.toSec(),4) / pow(T,4) - 
                            (6 * initial_pose_[i] - 6 * end_pose_[i]) * pow(elapsed_time_.toSec(),5) / pow(T,5);
      position_joint_handles_[i].setCommand(command_mat_joint[i]);
    }
  } else {
    for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(end_pose_[i]);
    }
  }

}

}  // namespace franka_panda_controller_swc

PLUGINLIB_EXPORT_CLASS(franka_panda_controller_swc::JointPositionController,
                       controller_interface::ControllerBase)
