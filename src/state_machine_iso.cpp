/********************************************************
Module for handling states during isometric and free motion tests
Assigns tasks, controls experimental state, communicates using the comms_iso module
Author: Joshua Rolls
Date: 30.08.21
********************************************************/

#include <franka_panda_controller_swc/state_machine_iso.h>


#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/robot.h>


namespace franka_panda_controller_swc {

/****************PUBLIC****************************************/

//initialise communication and states
bool StateMachineIsometric::init(Eigen::Vector3d initial_pos_d) {


    //setup triggers and things
    // node_handle.subscribe //do we feed the node handle in to the init?
    //trigger rate (need two rates)
    


    robot_position_d = initial_pos_d;
    protocol_state = ISO_STANDBY;
    standby_time = 0;
    timeAtTarget = 0;

    //init timers
    stateLoopTime = ctime::time();

    mode = FREE_MOTION_MODE;
    target_no = 0;
    target_pos = TARGETS_XYZ[target_no];

    // Comms_Hub.init(robot_position_d, protocol_state, target_no);
    contrComms.init(robot_position_d, protocol_state, mode, target_no);
    
    return true;
}

//run an iteration of the control/comms loop, sending information as necessary
bool StateMachineIsometric::update(Eigen::Vector3d pos_from_controller, Eigen::Vector3d force_from_controller) {

    //should be called by the controller every iteration
    //within this function we will look at triggers and things to manage publishing/subscribing
    //the update state functions don't have to happen super often
    //but publishing to isosim needs to be every iteration (but still deterministic!)

    //so.... 

    //update our recorded robot position
    set_robot_pos(pos_from_controller); //TODO: should this be in an if/else loop with the below?
    //TODO: do we need a set_robot_force() function? Maybe not...

    time_t simTime = ctime::time(); //TODO: fix time_t variable - if it's recording in seconds, that's a problem since we want to be looking at parts of a second...

    if (mode==ISOMETRIC_MODE) {

        //update the avatar position based on the latest data
        avatar_position = contrComms.get_latest_isosim_position();
        //we are sending force to isosim
        contrComms.publish_force(force_from_controller);
    }

    if (eventTimer(CONTROL_PERIOD,&stateLoopTime) ) { //TODO: implement actual trigger
        //we are doing our regular update
        
        if (mode == FREE_MOTION_MODE) {

            contrComms.publish_position(...);
        }
        
        //run state machine logic
    
        switch (protocol_state) {

            case ISO_STANDBY:

                if (standby_time == 0) {

                    standby_time = ctime::time();
                } else if (ctime::difftime(standby_time, ctime::time()) > STANDBY_PERIOD
                        && contrComms::check_comms_ack() ) {

                    //we've been in standby long enough and everything is online
                    //time to change states

                    standby_time = 0; //reset the standby counter for next time
                    std::cout << "START\n" ;
                    set_state(ISO_START);
                }

                break; //STANDBY

            case ISO_START:
                if (!contrComms::check_comms_ack()) {

                    //system is offline - go to stop mode
                    std::cout << "OFFLINE - STOP\n";
                    set_state(ISO_STOP);
                    break;
                }
                if(check_task_complete()) {

                    timeAtTarget = 0; //target is reached, reset for next time
                    completedTime = ctime::time();
                    std::cout <<"COMPLETE\n";
                    set_state(ISO_COMPLETE);

                };
                
                break;


            case ISO_STOP:
                //code
                //probably use this in case of disconnection problems or if the user provides input via cin (idk how that'd happen)
                //maybe if ack is received we can just go back to whatever we were doing before
                break;
            case ISO_COMPLETE:
                //code
                //task is complete, wait here for a few seconds
                if ( ctime::difftime(completedTime, ctime::time()) > COMPLETED_PERIOD ) {

                    //we've been in the completed zone long enough to celebrate, now on to the next!
                    if (handle_next_task() == true) {
                        
                        //standby before next task
                        std:cout << "next task, STANDBY\n";
                        set_state(ISO_STANDBY);
                    
                    } else {

                        //we've done everything. end of game
                        std::cout << "END EXPERIMENT\n";
                        set_state(ISO_END_EXPERIMENT);
                        return false;
                    }
                }
                break;
            
            case ISO_END_EXPERIMENT:
                //shouldn't be reached
                std::cout << "THE EXPERIMENT IS OVER. PLEASE SHUT EVERYTHING DOWN\n";
            default:
                std::cout << "ERROR: undefined state (in StateMachineIsometric::update()\n";

        }
        

    }

    return true;
    // TODO: finish update function
}

//-----------------setters-----------------

/*******
Sets recorded robot position based on input from robot sensors
Called inside update()
Returns true // TODO: error checking 
*******/
bool StateMachineIsometric::set_robot_pos(Eigen::Vector3d new_pos_d) {

    if (mode == FREE_MOTION_MODE) {

        avatar_position = new_pos_d;
    }

    return true;
}



//-----------------getters-----------------
Eigen::Vector3d StateMachineIsometric::get_robot_pos(void) {

    return robot_position_d;
}




/****************PRIVATE****************************************/

Eigen::Vector3d StateMachineIsometric::get_avatar_pos_iso(void) {

    //gets avatar position from comms hub
    // Comms_Hub::get_avatar_pos();
}




bool StateMachineIsometric::check_task_complete(void) {
    //checks if task has been complete for the required time within the required radius
    
    Eigen::Vector3d posErr = avatar_position - target_pos; //TODO: is this threadsafe?

    if (posErr.norm() < TARGET_RADIUS) {

        //in the zone

        if (timeAtTarget == 0) {
            
            //we only just reached the zone - now we wait TARGET_TIME
            timeAtTarget = ctime::time();
            
            return false;

        } else if ( ctime::difftime(ctime::time(),timeAtTarget) > TARGET_TIME ) {
            
            //we've been here long enough

            //TODO: add functionality to handle task complete
            //probably set a control flag saying "achieved", and send that in a control message at the end of the update loop
            return true;
        } 
        

    } else {


        //not in the zone
        timeAtTarget = 0;

    }

    return false   

}


bool StateMachineIsometric::handle_next_task(void) {

    //cycles between tasks/modes
    //free (1,2,3,4,5) --> iso (1,2,3,4,5) --> mix(1,2,3,4,5)
    //(mix not yet implemented)
    //if everything is done, return false
    //else return true
}

bool StateMachineIsometric::eventTimer(time_t period, time_t* prevTime) {

    time_t currTime = ctime::time();

    if (currTime - *prevTime >= period) {
        //at least one period has elapsed since this function last returned true
        *prevTime = currTime;
        return true;
    } else {
        return false;
    }
}

//////////////////////////////////////////////////////////////////
/*****************************************************************
 * ControllerComms functions and variables
 ****************************************************************/
 /////////////////////////////////////////////////////////////////

 
 bool ControllerComms::init(Eigen::Vector3d initial_avatar_pos,
        int iso_state, int iso_mode, int targ_no) {
    

     //start rosbridge

     //set params for initial message

     //publish first message (control)
     publish_control();

     //receive answer, hopefully
    //  subscribe_comms_ack();

     //done
    return true;

 }

Eigen::Vector3d ControllerComms::get_latest_isosim_position(void) {

    //TODO: needs to be threadsafe here...
    return _latest_pos
}


/**********PRIVATE******************************************/

bool ControllerComms::check_comms_ack(void){

    return _comms_ack; //TODO: needs to be threadsafe
}







} //namespace franka_panda_controller_swc