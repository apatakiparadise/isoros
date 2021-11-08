/**************************
* DEPRECATED - use state_machine_iso.cpp******
* Module to handle communications between
* controller, arm model, and unity
* using rosbridge
* Author: Joshua Rolls (with code adapted from Shih-Wen Chen and rosbridge examples)
* Date: 30.08.21
****************************/
#include <franka_panda_controller_swc/comms_iso.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/robot.h>




namespace franka_panda_controller_swc {



IsoCommunicator::init(Eigen::Vector3d initial_avatar_pos, int iso_state, int iso_mode, int targ_no) {

    avatar_pos = initial_avatar_pos;
    _state = iso_state;
    _mode = iso_mode;
    _target_no = targ_no;
    //initialise subscriber/publisher for unity and rtosim

    
}



}
