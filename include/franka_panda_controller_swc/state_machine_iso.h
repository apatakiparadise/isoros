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
#include <franka_panda_controller_swc/ForceOutput.h>
#include <franka_panda_controller_swc/ControlOutput.h>
#include <franka_panda_controller_swc/IsosimOutput.h>

#include <franka_panda_controller_swc/desired_mass_paramConfig.h>
#include <geometry_msgs/Vector3.h>

#include <ecl/threads/mutex.hpp>

#include <future>
#include <thread>
// #include <franka_panda_controller_swc/comms_iso.h>



#include <rosbridge_ws_client.hpp>

namespace franka_panda_controller_swc {

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
#define NUM_TARGET_POS 5 //number of positions
#define TARGET_RADIUS 5 // max distance from target to be considered achieved //TODO: what's the units here??
#define TARGET_TIME 5 //time in seconds that the avatar must be in the required zone for it to be considered complete


//wait triggers
#define STANDBY_PERIOD 10 //secs
#define COMPLETED_PERIOD 10 //secs

#define CONTROL_PERIOD 0.01 //secs, 100Hz

class StateMachineIsometric;
class ControllerComms;

        //structs within namespace
        struct ForceTime {
            
            Eigen::Vector3d force;
            double time;
        };

        struct ArmJointPosStruct {

            Eigen::Vector3d elbow;
            Eigen::Vector3d wrist;
            double time;
        };

        struct ControlInfo {

            ArmJointPosStruct position;
            double time;
            int target_no;
            bool reached;
        };

class localComms {

    public:
        bool init_local_comms(void);

        bool setForceData(ForceTime input);

        ArmJointPosStruct getArmPosFromIsosim(void);

    private:

        //THREADING STUFF
        std::promise<void> pubExitSignal;
        std::thread* pubTh;
        std::future<void> pubFuture;
        ecl::Mutex pubMutex;
        void forcePublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj);

        std::promise<void> subExitSignal;
        std::thread* subTh;
        std::future<void> subFuture;
        ecl::Mutex subMutex;
        void posSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message);
        void posSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj);

        //protected threading variables:
        ArmJointPosStruct _armData;
        bool _newPosAvailable = false;
        ForceTime _forceData; //holds the latest force data to be sent (x,y,z in Isosim's coordinates)
        bool _newForceAvailable = false; // indicates whether the values in _forceData have been published or not (true if new, false if previously pubbed)
        //protected function to publish data via rosbridgecpp
        void publishForce(ForceTime data);

        ArmJointPosStruct latestArmData; //threadsafe variable holding latest positions
};

//Handles communication within the controller thread
class ControllerComms {
    public:

        

        //initialises comms
        bool init(Eigen::Vector3d initial_avatar_pos, int iso_state, int iso_mode, int targ_no, ros::NodeHandle& handle);
        //publishes force to isosim
        bool publish_force(ForceTime forceToIsosim);

        //publishes control/position data to both isosim and commshub
        bool publish_control(ControlInfo control_info);
        ArmJointPosStruct get_latest_isosim_position(void); //threadsafe function for getting latest isosim position
        bool check_comms_ack(void); //threadsafe function for getting the state of the ack

        //init helpers
        void set_isosim_publish_rate(double rate);
        void set_comms_publish_rate(double rate);
        


    private:
        Eigen::Vector3d subscribe_isosim_position(void); //callback function. Sets a position variable that can be accessed by force-field functions, and published to the world
        bool subscribe_comms_ack(void); //can be fairly infrequent



        ArmJointPosStruct _latest_arm_pos; //holds latest position of arm joints (not threadsafe)
        ArmJointPosStruct current_arm_pos; //holds latest position of arm joints (threadsafe, should only be called within main thread)
        ecl::Mutex _arm_pos_mutex;

        ecl::Mutex _ack_mutex;
        bool _comms_ack; //tells whether system is online or not
        clock_t _last_comms_ack_time;

        //subscribers (isosim and communicator)
        ros::Subscriber sub_isosim_publisher_;
        ros::Subscriber sub_unity_subscriber_;
        void isosim_subscriber_callback(const franka_panda_controller_swc::ArmJointPosConstPtr& msg);
        void control_subscriber_callback(const geometry_msgs::Vector3ConstPtr& msg);

        // publishers (isosim and communicator)
        franka_hw::TriggerRate isosim_rate_trigger_{1.0};
        franka_hw::TriggerRate comms_rate_trigger_{1.0};
        realtime_tools::RealtimePublisher<ForceOutput> isosim_publisher_; //TODO figure out how to add this to /opt/ros/melodic/...
        realtime_tools::RealtimePublisher<ControlOutput> control_publisher_;

        localComms locals;
};


class StateMachineIsometric {

    public:
        bool init(Eigen::Vector3d initial_pos_d, ros::NodeHandle& handle);
        // setters
        bool set_robot_pos(Eigen::Vector3d new_pos_d); //called by ros controller
        // bool set_avatar_pos(Eigen::Vector3d new_pos_d); //called by either ControllerComms:subscribe_isosim_position() or by set_robot_pos() //unused, see update()
        
        // getters
        bool get_state(void);
        Eigen::Vector3d get_robot_pos(void);
        bool get_FF(void); //whether or not we are using the force field

        bool update(Eigen::Vector3d pos_from_controller, Eigen::Vector3d force_from_controller); //updates pos, checks if we've reached target

        

    private:
        Eigen::Vector3d robot_position_d; //actual position of franka

        //represented position of avatar (matches franka in free motion, based on isosim input in isometric)
        ArmJointPosStruct avatar_position; 
        ControlInfo latestControl;
        //set latest control
        void set_control_info(void);


        // IsoCommunicator Comms_Hub;
        ControllerComms contrComms;

        int protocol_state;
        bool mode; // free/iso
        bool FFon = false; //true if force field is activated, false if not
        int target_no;
        Eigen::Vector3d target_pos;
        clock_t standby_time; //Stores the time we first enter standby mode. 
                //Used to count how long we've been in ISO_STANDBY state. Set to 0 at init as well as whenever we leave ISO_STANDBY
        // clock_t startTime; //The time we start the simulation
        clock_t timeAtTarget; //time that the avatar has been consistently at the right target spot
        bool targetReached; //represents whether or not the task has been accomplished

        clock_t completedTime; //time that we completed the last task (used to check how long we've been in ISO_COMPLETE)


        Eigen::Vector3d TARGETS_XYZ[NUM_TARGET_POS] = {
          {1,1,0}, {1,2,0}, {2,1,0}, {2,2,0}, {3,1,0}
        }; //TODO

       

        Eigen::Vector3d get_avatar_pos_iso(void); //gets avatar position from comms during isometric exercises

        //state/mode functions
        void set_state(int state);
        void set_mode(bool mode); //sets free motion/iso in both robot controller and comms
        void set_FF(bool val);

        //task-related functions
        bool check_task_complete(void); //returns true if the position has been within TARGET_RADIUS for TARGET_TIME
        bool handle_next_task(void); //handles next task (cycles through targets and modes)


        //checks whether it's been the right amount of time since we executed the timer event
        bool eventTimer(double period, clock_t* prevTime);

        clock_t stateLoopTime;

        std::chrono::_V2::steady_clock::time_point stateLoopTime2;
        std::chrono::_V2::steady_clock::time_point startTime;
};
        
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





} //namespace franka_panda_controller_swc


#endif //STATE_MACHINE_ISO_H