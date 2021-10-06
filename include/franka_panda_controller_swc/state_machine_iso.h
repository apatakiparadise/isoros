/********************************************************
Module for handling states during isometric and free motion tests
Assigns tasks, controls experimental state, communicates using the comms_iso module
Author: Joshua Rolls
Date: 30.08.21
********************************************************/
#ifndef STATE_MACHINE_ISO_H
#define STATE_MACHINE_ISO_H

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ctime> //for state machine clocks

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

// #include <franka_panda_controller_swc/comms_iso.h>



// State definitions
#define ISO_STANDBY 0
#define ISO_START 1
#define ISO_STOP 2
#define ISO_COMPLETE 3
#define ISO_END_EXPERIMENT 4
//free/iso
#define FREE_MOTION_MODE 0
#define ISOMETRIC_MODE 1

//TODO: #def for target positions (or include an array of target positions?? could just be part of the class definition)
#define NUM_TARGET_POS 3
#define TARGET_RADIUS 5 // max distance from target to be considered achieved //TODO: what's the units here??
#define TARGET_TIME 5 //time in seconds that the avatar must be in the required zone for it to be considered complete


//wait triggers
#define STANDBY_PERIOD 10 //s
#define COMPLETED_PERIOD 10 //s

#define CONTROL_PERIOD 0.01 //100Hz

namespace franka_panda_controller_swc {


class StateMachineIsometric {

    public:
        bool init(Eigen::Vector3d initial_pos_d);
        // setters
        bool set_state(void);
        bool set_robot_pos(Eigen::Vector3d new_pos_d); //called by ros controller
        // bool set_avatar_pos(Eigen::Vector3d new_pos_d); //called by either ControllerComms:subscribe_isosim_position() or by set_robot_pos() //unused, see update()
        
        // getters
        bool get_state(void);
        Eigen::Vector3d get_robot_pos(void);

        bool update(Eigen::Vector3d pos_from_controller, Eigen::Vector3d force_from_controller); //updates pos, checks if we've reached target

    private:
        Eigen::Vector3d robot_position_d; //actual position of franka

        //represented position of avatar (matches franka in free motion, based on isosim input in isometric)
        Eigen::Vector3d avatar_position; 
        
        // IsoCommunicator Comms_Hub;
        ControllerComms contrComms;

        int protocol_state;
        bool mode; // free/iso
        int target_no;
        time_t standby_time; //Stores the time we first enter standby mode. 
                //Used to count how long we've been in ISO_STANDBY state. Set to 0 at init as well as whenever we leave ISO_STANDBY
        time_t timeAtTarget; //time that the avatar has been consistently at the right target spot
        bool targetReached; //represents whether or not the task has been accomplished

        time_t completedTime; //time that we completed the last task (used to check how long we've been in ISO_COMPLETE)


        Eigen::Vector3d TARGETS_XYZ[NUM_TARGET_POS] = {
          (1,1,0), (1,2,0), (2,1,0), (2,2,0), (3,1,0)
        }; //TODO

        bool set_mode(bool); //sets free motion/iso in both robot controller and comms

        Eigen::Vector3d get_avatar_pos_iso(void); //gets avatar position from comms during isometric exercises

        //state/mode functions
        void set_state(int state);
        void set_mode(bool mode);

        //task-related functions
        bool check_task_complete(void); //returns true if the position has been within TARGET_RADIUS for TARGET_TIME
        bool handle_next_task(void); //handles next task (cycles through targets and modes)


        //checks whether it's been the right amount of time since we executed the timer event
        bool eventTimer(int period, time_t* prevTime);

        time_t stateLoopTime;
}
        
/*
//TODO: figure out a better system for multi-tasking and updating
        probably don't need to be updating state machine every time ros updates (1kHz)
        maybe put a trigger in the force_controller to call update occasionally
        In this case you'd need the NR controller to be publishing directly, more often than we're checking things
        I guess controller_NR can call a comms function to publish often (1kHz to rtosim, less often for unity)
        Then at maybe 100Hz we can update the position in the state machine and run some logic there to decide whether they have reached the target or not
        If they've reached the target (or some other end condition), StateMachineIsometric::get_state will return a paused/stopped state.
        This can be used as a condition in the 1kHz publishing logic.
        Will need to make sure these extra classes don't slow everything down.

//TODO: Remember that the comms thread will have to be a separately-compiled cpp program to handle slower communication between ros and unity
        I thing that's right
        Might be okay if comms was done in real-time, just don't want to slow down ros by doing too much all at once
        I guess it could all be done with counters to make sure it doesn't happen too often.......
        
        Otherwise we'll need to be doing continuous publishing from controller to comms thread using rosbridge
        which would enable controller/state machine to keep going fast without having to worry about too much logic related to communications from unity
        I don't know if that will solve too much though, since the state machine logic will still be relying on subscribed data.


*/

}

//Handles communication within the controller thread
class ControllerComms {
    public:
        //initialises comms
        bool init(Eigen::Vector3d initial_avatar_pos, int iso_state, int iso_mode, int targ_no);
        //publishes force to isosim
        bool publish_force(Eigen::Vector3d force_to_isosim);
        //publishes franka position to commshub
        bool publish_position(Eigen::Vector3d pos_to_commshub);
        //publishes control data to both isosim and commshub
        bool publish_control(Eigne::Vector3d control_to_all);
        Eigen::Vector3d get_latest_isosim_position(void); //threadsafe function for getting latest isosim position
        bool check_comms_ack(void); //threadsafe function for getting the state of the ack

    private:
        Eigen::Vector3d subscribe_isosim_position(void); //callback function. Sets a position variable that can be accessed by force-field functions, and published to the world
        bool subscribe_comms_ack(void); //can be fairly infrequent


        Eigen::Vector3d _latest_pos; //holds latest position, threadsafe
        bool _comms_ack; //tells whether system is online or not






} //namespace franka_panda_controller_swc


#endif //STATE_MACHINE_ISO_H