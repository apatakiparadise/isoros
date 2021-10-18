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

#include <ctime>


double _clock_secs(clock_t ctim);

namespace franka_panda_controller_swc {

/****************PUBLIC****************************************/

//initialise communication and states
bool StateMachineIsometric::init(Eigen::Vector3d initial_pos_d, ros::NodeHandle& handle) {


    //setup triggers and things
    // node_handle.subscribe //do we feed the node handle in to the init?
    //trigger rate (need two rates)
    


    robot_position_d = initial_pos_d;
    protocol_state = ISO_STANDBY;
    standby_time = 0;
    timeAtTarget = 0;

    //init timers
    stateLoopTime = std::clock();
    startTime = std::chrono::steady_clock::now();

    mode = ISOMETRIC_MODE;
    if (!handle.getParam("isometric",mode)) {
        ROS_INFO_STREAM("StateMachine: isometric/free-motion not found. Defaulting to isometric");
    }

    target_no = 0;
    target_pos = TARGETS_XYZ[target_no];

    // Comms_Hub.init(robot_position_d, protocol_state, target_no);
    contrComms.init(robot_position_d, protocol_state, mode, target_no, handle);
    
    return true;
}

//run an iteration of the control/comms loop, sending information as necessary
bool StateMachineIsometric::update(Eigen::Vector3d pos_from_controller, Eigen::Vector3d force_from_controller) {

    //should be called by the controller every iteration
    //within this function we will look at triggers and things to manage publishing/subscribing
    //the update state functions don't have to happen super often
    //but publishing to isosim needs to be every iteration (but still deterministic!)

    //so.... 
    //update our recorded robot EE position

    set_robot_pos(pos_from_controller); //TODO: should this be in an if/else loop with the below?
    //TODO: do we need a set_robot_force() function? Maybe not...

    auto chronoTime =  std::chrono::steady_clock::now();
    std::chrono::duration<double> simTimeChrono = std::chrono::duration_cast<std::chrono::duration<double>>(chronoTime - startTime);
    double simTime = simTimeChrono.count();
    // double simTime = _clock_secs(std::clock() - startTime); //TODO: fix time_t variable - if it's recording in seconds, that's a problem since we want to be looking at parts of a second...
    

    if (mode==ISOMETRIC_MODE) {

        //update the avatar position based on the latest data
        avatar_position = contrComms.get_latest_isosim_position();
        //we are sending force to isosim
        ForceTime forceTime_;
        forceTime_.force = force_from_controller;
        forceTime_.time = simTime;
        contrComms.publish_force(forceTime_);
    }

    if (eventTimer(CONTROL_PERIOD,&stateLoopTime) ) { //TODO: implement actual trigger
        //we are doing our regular update
        
 
        //run state machine logic
    
        switch (protocol_state) {

            case ISO_STANDBY:

                if (standby_time == 0) {

                    standby_time = std::clock();
                } else if (   _clock_secs(std::clock() - standby_time) > STANDBY_PERIOD
                        && contrComms.check_comms_ack() ) {

                    //we've been in standby long enough and everything is online
                    //time to change states

                    standby_time = 0; //reset the standby counter for next time
                    std::cout << "START\n" ;
                    set_state(ISO_START);
                }

                break; //STANDBY

            case ISO_START:
                if (!contrComms.check_comms_ack()) {

                    //system is offline - go to stop mode
                    std::cout << "OFFLINE - STOP\n";
                    set_state(ISO_STOP);
                    break;
                }
                if(check_task_complete()) {

                    timeAtTarget = 0; //target is reached, reset for next time
                    completedTime = std::clock();
                    std::cout <<"COMPLETE\n";
                    set_state(ISO_COMPLETE);

                };
                
                break; //ISO_START


            case ISO_STOP:
                //code
                //probably use this in case of disconnection problems or if the user provides input via cin (idk how that'd happen)
                //maybe if ack is received we can just go back to whatever we were doing before
                if (contrComms.check_comms_ack()) {
                    set_state(ISO_START);
                }
                break;
            case ISO_COMPLETE:
                //code
                //task is complete, wait here for a few seconds
                if ( _clock_secs(std::clock() - completedTime) > COMPLETED_PERIOD ) {

                    //we've been in the completed zone long enough to celebrate, now on to the next!
                    if (handle_next_task() == true) {
                        
                        //standby before next task
                        std::cout << "next task, STANDBY\n";
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
        set_control_info(); //update control struct
        // latestControl.position.wrist = {1,2,3};
        contrComms.publish_control(latestControl);

    }

    return true;
    // TODO: finish update function
}

//-----------------setters-----------------

//sets state (run/stop etc)
void StateMachineIsometric::set_state(int state) {

    protocol_state = state;
    return;
}

/*******
Sets recorded robot EE position based on input from robot sensors
Called inside update()
Returns true // TODO: error checking 
*******/
bool StateMachineIsometric::set_robot_pos(Eigen::Vector3d new_pos_d) {

    if (mode == FREE_MOTION_MODE) {

        avatar_position.wrist = new_pos_d;
    }

    return true;
}



//-----------------getters-----------------

bool StateMachineIsometric::get_FF(void) {

    return FFon;
}

Eigen::Vector3d StateMachineIsometric::get_robot_pos(void) {

    return robot_position_d;
}




/****************PRIVATE****************************************/

void StateMachineIsometric::set_control_info(void) {

    latestControl.position = avatar_position; //should be up to date with the latest position already 
                                            // (either set by get_latest_isosim_position() in iso or by set_robot_pos() in free motion)
    if (protocol_state == ISO_COMPLETE) {

        latestControl.reached = true;
    } else {
        latestControl.reached = false;
    }
    latestControl.target_no = target_no;
    latestControl.time = latestControl.position.time;
}

Eigen::Vector3d StateMachineIsometric::get_avatar_pos_iso(void) {

    //gets avatar position from comms hub
    // Comms_Hub::get_avatar_pos();
}

void StateMachineIsometric::set_FF(bool val) {
    
    FFon = val;
    return;

}


bool StateMachineIsometric::check_task_complete(void) {
    //checks if task has been complete for the required time within the required radius
    
    Eigen::Vector3d posErr = avatar_position.wrist - target_pos; //TODO: is this threadsafe?

    if (posErr.norm() < TARGET_RADIUS) {

        //in the zone

        if (timeAtTarget == 0) {
            
            //we only just reached the zone - now we wait TARGET_TIME
            timeAtTarget = std::clock();
            
            return false;

        } else if ( _clock_secs(std::clock() - timeAtTarget) > TARGET_TIME ) {
            
            //we've been here long enough

            //TODO: add functionality to handle task complete
            //probably set a control flag saying "achieved", and send that in a control message at the end of the update loop
            return true;
        } 
        

    } else {


        //not in the zone
        timeAtTarget = 0;

    }

    return false;

}


bool StateMachineIsometric::handle_next_task(void) {

    //cycles between tasks/modes
    //free (1,2,3,4,5) --> iso (1,2,3,4,5) --> mix(1,2,3,4,5)
    //should set the force field or no (for familiarisation)
    //(mix not yet implemented)
    //if everything is done, return false
    //else return true
}

bool StateMachineIsometric::eventTimer(double period, clock_t* prevTime) {

    clock_t currTime = std::clock();

    if ( _clock_secs(currTime - *prevTime) >= period) {
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

 /*
 Initialise comms
 rate triggers should already have been set by controller
    node handle params using the set() functions of this class

 */
 bool ControllerComms::init(Eigen::Vector3d initial_avatar_pos,
        int iso_state, int iso_mode, int targ_no, ros::NodeHandle& handle) {
    

     //start rosbridge
    isosim_publisher_.init(handle, "force_output",1);
    control_publisher_.init(handle, "control_output",1);

    //subscribers
    sub_isosim_publisher_ = handle.subscribe("/isosimtopic", 20, &ControllerComms::isosim_subscriber_callback,  this, ros::TransportHints().reliable().tcpNoDelay());
    sub_unity_subscriber_ = handle.subscribe("/unity_output",20,&ControllerComms::control_subscriber_callback, this, ros::TransportHints().reliable().tcpNoDelay());
     //set params for initial message
    ControlInfo initialControl;
    initialControl.position.wrist = initial_avatar_pos;
    initialControl.position.elbow = initial_avatar_pos - Eigen::Vector3d(1,0,0);
    initialControl.time = 0; // (TODO: get time from somewhere)
    initialControl.position.time = initialControl.time;
    initialControl.reached = false;
    initialControl.target_no = 0;
    
    //publish first message (control)
    publish_control(initialControl);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    locals.init_local_comms();

     //receive answer, hopefully
    //  subscribe_comms_ack();

     //done
    return true;

 }

bool ControllerComms::publish_force(ForceTime forceToIsosim) {

    //coordinate transform (ros to opensim):
        // x becomes x
        // y becomes -z
        // z becomes -y

    ForceTime output;
    // output.force.x() = forceToIsosim.force.x();
    // output.force.y() = - forceToIsosim.force.z();
    // output.force.z() = - forceToIsosim.force.y();

    output.force = {forceToIsosim.force.x(),  - forceToIsosim.force.z(),  - forceToIsosim.force.y()};

    output.time = forceToIsosim.time;
    if (locals.setForceData(output) == true) {
        return true;
    } else {
        return false;
    };

    //NOT REACHED (redundant code from previous example)
    if (isosim_publisher_.trylock()) {
        isosim_publisher_.msg_.force.x = forceToIsosim.force.x();
        isosim_publisher_.msg_.force.y = - forceToIsosim.force.z();
        isosim_publisher_.msg_.force.z = - forceToIsosim.force.y();
        isosim_publisher_.msg_.time = forceToIsosim.time;

        isosim_publisher_.unlockAndPublish();

        return true;
    } else {
        return false;
    }
}


//publishes control information
bool ControllerComms::publish_control(ControlInfo info) {
    
    //coordinate transform (ros to opensim): //TODO: change this to the Unity coordinate transform
        // x becomes x
        // y becomes -z
        // z becomes -y
    if (control_publisher_.trylock()) {
        control_publisher_.msg_.avatarpos.elbow.x = + info.position.elbow.x();
        control_publisher_.msg_.avatarpos.elbow.y = - info.position.elbow.z();
        control_publisher_.msg_.avatarpos.elbow.z = - info.position.elbow.y();
        control_publisher_.msg_.avatarpos.wrist.x = + info.position.wrist.x();
        control_publisher_.msg_.avatarpos.wrist.y = - info.position.wrist.z();
        control_publisher_.msg_.avatarpos.wrist.z = - info.position.wrist.y();
        control_publisher_.msg_.avatarpos.time = info.position.time;
        control_publisher_.msg_.reached = info.reached;
        control_publisher_.msg_.targetno = info.target_no;
        control_publisher_.msg_.time = info.time;

        control_publisher_.unlockAndPublish();

        return true;
    } else {
        return false;
    }
}

/* Threadsafe function for getting the latest arm position received from isosim
    Tries to unlock mutex for arm variable. If unsuccessful, will just return previous value
*/
ArmJointPosStruct ControllerComms::get_latest_isosim_position(void) {

    current_arm_pos = locals.getArmPosFromIsosim();

    return current_arm_pos;


    ///NOT REACHED used by subscriber over network (currently inactive)
    if (_arm_pos_mutex.trylock()) {
        current_arm_pos = _latest_arm_pos;
        _arm_pos_mutex.unlock();
    }

    return current_arm_pos;
}

void ControllerComms::set_isosim_publish_rate(double rate) {

    isosim_rate_trigger_ = franka_hw::TriggerRate(rate);
}

void ControllerComms::set_comms_publish_rate(double rate) {

    comms_rate_trigger_ = franka_hw::TriggerRate(rate);
}

/**********PRIVATE******************************************/

bool ControllerComms::check_comms_ack(void){

    return true;
    bool ack_ = false;
    if (_ack_mutex.trylock()) {
        ack_ = _comms_ack;
        _ack_mutex.unlock();
    }
    return ack_;

    // return _comms_ack; //TODO: needs to be threadsafe
}



/* called as part of subscriber thread*/
void ControllerComms::isosim_subscriber_callback(const franka_panda_controller_swc::ArmJointPosConstPtr& msg) {
    //TODO: handle transform between isosim and ros coordinates

    _arm_pos_mutex.lock(); //will throw error if mutex is not lockable

    _latest_arm_pos.time = msg->time;
    _latest_arm_pos.elbow.x() = msg->elbow.x;
    _latest_arm_pos.elbow.y() = msg->elbow.y;
    _latest_arm_pos.elbow.z() = msg->elbow.z;
    _latest_arm_pos.wrist.x() = msg->wrist.x;
    _latest_arm_pos.wrist.y() = msg->wrist.y;
    _latest_arm_pos.wrist.z() = msg->wrist.z;
    
    
    _arm_pos_mutex.unlock();
}


void ControllerComms::control_subscriber_callback(const geometry_msgs::Vector3ConstPtr& msg) {

    //TODO: implement comms from unity to control (should just be an ack?)
    Eigen::Vector3d vecIn = {msg->x,msg->y,msg->z};
    {
        _ack_mutex.lock();
        _comms_ack = (bool) vecIn.maxCoeff();
    }
    return;
}

//global within namespace
RosbridgeWsClient RBclient("localhost:9090");


void advertiserCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message) {

}

bool localComms::init_local_comms(void)  {

    RBclient.addClient("service_advertiser");
    RBclient.advertiseService("service_advertiser", "/controllerservice", "std_srvs/SetBool", &advertiserCallback);

    // RBclient.addClient("topic_advertiser");
    // RBclient.advertise("topic_advertiser", "/ROSforceOutput", "franka_panda_controller_swc/ForceOutput");

    subFuture = subExitSignal.get_future();
    subTh = new std::thread(&localComms::forcePublisherThread, this, std::ref(RBclient), std::cref(subFuture));

    pubFuture = pubExitSignal.get_future();
    pubTh = new std::thread(&localComms::posSubscriberThread, this, std::ref(RBclient),std::cref(subFuture));
}

bool localComms::setForceData(ForceTime input) {
    if (pubMutex.trylock()) {
        _forceData = input;
        _newForceAvailable = true; //note: we don't check this flag here, so any un-used data will be overwritten
        pubMutex.unlock();
        return true;
    } else {
        return false;
    }

}

/*threadsafe method to obtain the arm position saved in _armData as received from /isosimtopic*/
ArmJointPosStruct localComms::getArmPosFromIsosim(void) {

    if (subMutex.trylock()) {
        latestArmData = _armData;
        subMutex.unlock();
    }
    //threadsafe
    return latestArmData; //if the mutex is locked, we'll just return the previous value

}

/////PRIVATE FUNCTIONS and threading
void localComms::forcePublisherThread(RosbridgeWsClient& client, const std::future<void>& futureObj) {

    ROS_INFO_STREAM("FORCE PUBLISHER THREAD BEGIN");

    client.addClient("topic_advertiser");
    client.advertise("topic_advertiser", "/ROSforceOutput", "franka_panda_controller_swc/ForceOutput");


    client.addClient("force_publisher");

    ForceTime forceToSend;
    bool newData = false;
    while(futureObj.wait_for(std::chrono::milliseconds(2)) == std::future_status::timeout) {

        if (pubMutex.trylock()) {
            if (_newForceAvailable) {
                forceToSend = _forceData;
                newData = true;
                _newForceAvailable = false; //we've accessed the most recent data
            }
            pubMutex.unlock();
        }
        if (newData) {
            publishForce(forceToSend);
            newData = false;
        }
        //continue running until signal received
    }
    client.removeClient("force_publisher");
    ROS_INFO_STREAM("FORCE PUBLISHER OUT");
}

/* Function to publish force to a topic. Should be run within the publishing thread */
void localComms::publishForce(ForceTime data) {

    // std::cout << "force out " << data.force << " time " << data.time << std::endl;
 

    rapidjson::Document d;

    d.SetObject();

    rapidjson::Value msg(rapidjson::kObjectType);

    rapidjson::Value fVec(rapidjson::kObjectType);
    rapidjson::Value fx;
    rapidjson::Value fy;
    rapidjson::Value fz;
    fx.SetDouble(data.force.x());
    fy.SetDouble(data.force.y());
    fz.SetDouble(data.force.z());
    
    fVec.AddMember("x", fx, d.GetAllocator());
    fVec.AddMember("y", fy, d.GetAllocator());
    fVec.AddMember("z", fz, d.GetAllocator());


    rapidjson::Value timestamp;
    timestamp.SetDouble(data.time);

    d.AddMember("force",fVec,d.GetAllocator());
    d.AddMember("time",timestamp,d.GetAllocator());

    RBclient.publish("/ROSforceOutput",d);

    return;

}

ecl::Mutex subMutex;

void localComms::posSubscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::InMessage> in_message) {

    rapidjson::Document d;

  
    if (d.Parse(in_message->string().c_str()).HasParseError() ) {
        std::cerr << "\n\nparse error\n" << std::endl;
    };

    assert(d.IsObject());    // Document is a JSON value represents the root of DOM. Root can be either an object or array.
    assert(d.HasMember("msg"));
    assert(d["msg"].HasMember("wrist"));
    assert(d["msg"].HasMember("elbow"));
    assert(d["msg"]["wrist"].HasMember("x"));
    assert(d["msg"].HasMember("time"));

    double wx = d["msg"]["wrist"]["x"].GetDouble();
    double wy = d["msg"]["wrist"]["y"].GetDouble();
    double wz = d["msg"]["wrist"]["z"].GetDouble();

    double ex = d["msg"]["elbow"]["x"].GetDouble();
    double ey = d["msg"]["elbow"]["y"].GetDouble();
    double ez = d["msg"]["elbow"]["z"].GetDouble();

    double timestamp = d["msg"]["time"].GetDouble();
    std::cout << "wx is " << wx << std::endl;
    subMutex.lock(); //lock mutex
        _armData.wrist = {wx,wy,wz};
        _armData.elbow = {ex,ey,ez};
        _armData.time = timestamp;
        _newPosAvailable = true;
        std::cout << "arm pos received: " << _armData.wrist << std::endl;
    subMutex.unlock();

}


void localComms::posSubscriberThread(RosbridgeWsClient& client, const std::future<void>& futureObj) {

    RBclient.addClient("topic_subscriber");

    std::function<void(std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message)> _posCallbackFunc;
    _posCallbackFunc = [this](std::shared_ptr<WsClient::Connection> connection, std::shared_ptr<WsClient::InMessage> in_message){this->posSubscriberCallback(connection, in_message);};
    RBclient.subscribe("topic_subscriber","/isosimtopic",_posCallbackFunc);

    while(futureObj.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
        //do nothing (everything is handled by the callbacks
    }

}





} //namespace franka_panda_controller_swc

//PRIVATE HELPER FUNCTIONS

/*converts clock ticks into seconds */
double _clock_secs(clock_t ctim) {
    double tim = (double) ctim;
    double cps =  CLOCKS_PER_SEC;
    double res = tim/cps;
    

    return res;


    // return (double) ((double) ctim) / ((double) CLOCKS_PER_SEC) ;
}