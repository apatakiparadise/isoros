/**************************
* Module to handle communications between
* controller, arm model, and unity
* using rosbridge
* Author: Joshua Rolls (with code adapted from Shih-Wen Chen and rosbridge examples)
* Date: 30.08.21
****************************/
#ifndef COMMS_ISO_H
#define COMMS_ISO_H

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


class IsoCommunicator {

    public:
        bool init(Eigen::Vector3d initial_avatar_pos, int iso_state, int iso_mode, int target_no);

        bool set_avatar_pos_comms(Eigen::Vector3d); //sets the avatar position

        Eigen::Vector3d get_avatar_pos_comms(void); //returns position of avatar (using data received from rtosim)
        
    private:
        // Subscribe from Unity
        ros::Subscriber sub_unity_publisher_;
        void Unity_publisher(const geometry_msgs::Vector3ConstPtr& msg);

        // Publisher to Unity
        franka_hw::TriggerRate rate_trigger_{1.0};
        realtime_tools::RealtimePublisher<UnityInput> unity_publisher_;

        Eigen::Vector3d avatar_pos;

        int _state;
        int _mode;
        int _target_no;


}



}



#endif //COMMS_ISO_H 