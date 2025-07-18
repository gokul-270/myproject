/*  Author: Saurabh Bansal

Desc:   O-Drive Hardware control, to be used with ros-control
*/
#include <unistd.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/SetBool.h>

#include <odrive_control/generic_hw_interface.h>
#include <odrive_control/odrive_hw_interface.h>
#include <dynamixel_msgs/JointState.h>
// #include <wiringPi.h>
#include <pigpiod_if2.h>

bool height_scan_enable = false;
int pi;
// added by ribin for seting a flag while initilaisation so that the read and write function wont be called during the initilaisation
int Block_Read_Write_During_Initialization = 1; // Default set it to 1 (until the initialisation is done
namespace odrive_control
{
    ODriveHWInterface::ODriveHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        // Getting the number of Joints, as num_joints_ will be updated in init function
        size_t num_joints = joint_names_.size();
        size_t num_joints_ = joint_names_.size();

        // Initialise the serial interfaces for O-Drives
        nh_.param<int>("/hardware_interface/baud", baud_rate_, 115200);
        nh_.getParam("/hardware_interface/odrive_0", odrive_serial_number_[0]);
        nh_.getParam("/hardware_interface/odrive_1", odrive_serial_number_[1]);
        // added by ribin on 10/03/2020 for clearing the stol error_code
        nh_.param<bool>("joint2_init/height_scan_enable", height_scan_enable, false);

        odrive_serial_port_[0] = new comm_serial(odrive_serial_number_[0], baud_rate_);
        odrive_serial_port_[1] = new comm_serial(odrive_serial_number_[1], baud_rate_);

        // Take objects of class odrive_control, of count 'num_joints'
        odrive_serial_interface_.resize(num_joints);
        odrive_id_.resize(num_joints);
        axis_id_.resize(num_joints);
        limit_switch_id_.resize(num_joints);
        transmission_factor_.resize(num_joints);
        encoder_resolution_.resize(num_joints);
        direction_.resize(num_joints);
        p_gain_.resize(num_joints);
        v_gain_.resize(num_joints);
        v_int_gain_.resize(num_joints);
        zero_offset_.resize(num_joints, 0.0);
        //	zero_offset_home_.resize(num_joints, 0.0);
        temp_threshold_.resize(num_joints);
        error_threshold_.resize(num_joints);
        current_threshold_.resize(num_joints);
        max_velocity_.resize(num_joints);
        min_velocity_.resize(num_joints);
        switch_id_.resize(num_joints);
        reference_target_.resize(num_joints);
        homing_position_.resize(num_joints);
        state_.resize(num_joints);
        last_joint_position_command_.resize(num_joints);
        //joint_state_pub_.resize(num_joints);
        joint_state_subscriber_.resize(num_joints);
        joint_homing_switch_in_.resize(num_joints);
        No_Of_Joint_initialised_to_home_ = 0;
        homing_pos_init_seq_.resize(num_joints);

        //ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", num_joints_);
        //joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", num_joints_);
        //ODriveJointStatePublisher.resize(num_joints_);
        //joint_state_.position.resize(num_joints_);
        //joint_state_.name.resize(num_joints_);

        // sensor_msgs::JointState joint_state;
        joint_state.name.resize(num_joints_);
        joint_state.position.resize(num_joints_);
        //joint_state.name[joint_id] = joint_names_[joint_id];
        // Check whether user has requested for reconfiguration of O-Drives from yaml file
        nh_.param("hardware_interface/odrive_reconfigure_request", odirve_configuration_required_, false);
        std::string controller_names = joint_names_[0] + "_position_controller/";

        nh_.getParam(controller_names + "homing_pos_init_seq_", homing_pos_init_seq_[0]); // added by ribin for initialisation of joint sequance without assistance
        // initalising pigpiod
        ROS_INFO("initalising pigpiod");
        pi = pigpio_start(NULL, NULL);
        if (pi < 0)
        {
            ROS_ERROR("pigpio_start() function did not successful");
            // exit(1) ;
        }
        // Initialisation of all Joint Actuators
        for (std::size_t joint_id = 0; joint_id < num_joints; ++joint_id)
        {
            // Read the Joint's parameters

            std::string controller_name = joint_names_[joint_id] + "_position_controller/";
            //std::string controller_name = joint_names_[joint_id] ;
            ROS_INFO("number of joints_present := %d", (int)num_joints);
            ROS_INFO("Present Joint := %s", controller_name.c_str());

            nh_.getParam(controller_name + "odrive_id", odrive_id_[joint_id]);
            nh_.getParam(controller_name + "axis_id", axis_id_[joint_id]);
            nh_.getParam(controller_name + "transmission_factor", transmission_factor_[joint_id]);
            nh_.getParam(controller_name + "resolution", encoder_resolution_[joint_id]);
            nh_.getParam(controller_name + "direction", direction_[joint_id]);
            nh_.getParam(controller_name + "p_gain", p_gain_[joint_id]);
            nh_.getParam(controller_name + "v_gain", v_gain_[joint_id]);
            nh_.getParam(controller_name + "v_int_gain", v_int_gain_[joint_id]);
            nh_.getParam(controller_name + "max_t", temp_threshold_[joint_id]);
            nh_.getParam(controller_name + "max_e", error_threshold_[joint_id]);
            nh_.getParam(controller_name + "max_cur", current_threshold_[joint_id]);
            nh_.getParam(controller_name + "max_vel", max_velocity_[joint_id]);
            nh_.getParam(controller_name + "min_vel", min_velocity_[joint_id]);
            nh_.getParam(controller_name + "switch", switch_id_[joint_id]);
            nh_.getParam(controller_name + "limit_switch", limit_switch_id_[joint_id]);
            nh_.getParam(controller_name + "reference_trgt", reference_target_[joint_id]);
            nh_.getParam(controller_name + "homing_pos", homing_position_[joint_id]);
            last_joint_position_command_[joint_id] = 0.0; // TODO: Check if it is right value

            ROS_INFO( " odrive_id : %d ", (int) odrive_id_[joint_id]);
            ROS_INFO( "axis_id : %d ",  (int) axis_id_[joint_id]);
            ROS_INFO( "transmission_factor : %d ", (int)  transmission_factor_[joint_id]);
            ROS_INFO( "resolution : %d ", (int)  encoder_resolution_[joint_id]);
            ROS_INFO( "direction : %d ", (int)  direction_[joint_id]);
            ROS_INFO( "p_gain : %f ", (float)  p_gain_[joint_id]);
            ROS_INFO( "v_gain : %f ",  (float)  v_gain_[joint_id]);
            ROS_INFO( "v_int_gain : %f ",  (float)  v_int_gain_[joint_id]);
            ROS_INFO( "max_t : %f ",  (float)  temp_threshold_[joint_id]);
            ROS_INFO( "max_e : %f ",  (float)  error_threshold_[joint_id]);
            ROS_INFO( "max_cur : %f ",  (float)  current_threshold_[joint_id]);
            ROS_INFO( "max_vel : %f ",  (float)  max_velocity_[joint_id]);
            ROS_INFO( "min_vel : %f ",  (float)  min_velocity_[joint_id]);
            //ROS_INFO( "switch : %s",   switch_id_[joint_id]); // Formating is wrong for the string
            ROS_INFO( "limit_switch : %d ", (int)  limit_switch_id_[joint_id]);
            ROS_INFO( "reference_trgt : %f ",  (float)  reference_target_[joint_id]);
            ROS_INFO( "homing_po : %f ",  (float)  homing_position_[joint_id]);
            last_joint_position_command_[joint_id] = 0.0; // TODO: Check if it is right value




            // Initialise the O-Drive Serial Interface
            odrive_serial_interface_[joint_id] = new ODriveSerial(*odrive_serial_port_[odrive_id_[joint_id]],
                    axis_id_[joint_id]);
            ROS_INFO("Initialised O-Drive Serial Interface for %s which is at axis_id_[joint_id]:%d", joint_names_[joint_id].c_str(), (int)axis_id_[joint_id]);
            // If O-Drive configuration is required from configuration .ymal file
            if (odirve_configuration_required_ == true)
            {
                // TODO: We should be able to Read the motor calibration parameters and write them also
                // Initialise the parameters for this Joint
                odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POS_GAIN_RW, p_gain_[joint_id]);
                odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VEL_GAIN_RW, v_gain_[joint_id]);
                odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VEL_INTEGRATOR_GAIN_RW, v_int_gain_[joint_id]);
                odrive_serial_interface_[joint_id]->WriteProperty(ODRV_CURRENT_LIM_RW, current_threshold_[joint_id]);
                // Set the Max. Valocity the Joint can reach, convert from rpm to count/sec
                odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW,
                        (max_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id]) / 60);
            }
            // Publishing the Joint States messages to be used by yanthra_move
            ROS_INFO("Debug Position 0 d");
            // Changed from 1000 to 5000 on Aug16 to move more angle of the arm during initialisation
            //joint_state_[joint_id] = nh_.advertise<dynamixel_msgs::JointState>(controller_name + "state", 10);
            //joint_state_.name[joint_id]= controller_name + "/state";
            joint_state.name[joint_id] = joint_names_[joint_id];
            //ODriveJointStatePublisher[joint_id].name =  controller_name + "/state";
            //ODriveJointStatePublisher[joint_id].Publisher = nh_.advertise<std_msgs::Float64>(ODriveJointStatePublisher[joint_id].name , 1);
            // Commented out the following "joint_states" publisher : 6 MARCH 2023 Author Manohar Sambandam
            joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

            // Subscribe to joint command here
            //ros::Subscriber Joint2Subscriber,Joint3Subscriber,Joint4Subscriber,Joint5Subscriber;
            if(joint_id == 0) {
               Joint3Subscriber= nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id]+"_position_controller/command", 1, &ODriveHWInterface::Joint3PositionCallBack, this);
               ROS_INFO("Subscribed  Joint3PositionCallBack to joint_id : %d",   (int) joint_id) ;
            }
            if(joint_id == 1) { Joint4Subscriber= nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id]+"_position_controller/command", 1, &ODriveHWInterface::Joint4PositionCallBack, this);

               ROS_INFO("Subscribed  Joint4PositionCallBack to joint_id : %d",   (int) joint_id) ;
            }
            if(joint_id == 2) { Joint5Subscriber= nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id]+"_position_controller/command", 1, &ODriveHWInterface::Joint5PositionCallBack, this);

               ROS_INFO("Subscribed  Joint5PositionCallBack to joint_id : %d",  (int) joint_id) ;
            }
            if(joint_id == 3) { Joint2Subscriber= nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id]+"_position_controller/command", 1, &ODriveHWInterface::Joint2PositionCallBack, this);

               ROS_INFO("Subscribed  Joint2PositionCallBack to joint_id : %d",  (int) joint_id) ;
            }

            // Service Client for Joint Homing Limit Switch
            // TODO: Put a check whether services are available
            joint_homing_switch_in_[joint_id] = nh_.serviceClient<std_srvs::SetBool>(switch_id_[joint_id]);
            ROS_INFO("Debug Position 1");

            // Check if there is some Axis Error
            long response = odrive_serial_interface_[joint_id]->GetError();
            if (response != 0)
            {
                ROS_ERROR("%s: Axis Error Status: %ld", joint_names_[joint_id].c_str(), response);
            }
            // odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_IDLE, STATE_RQST_WAIT);
            ROS_INFO("Debug Position 3 ");

            // chaned by ribin on 10/03/2020
            // Changing the sequance of initialisation to l2 first so moved this poriton of the code first
            ROS_INFO("SETTING THE PIN FOR %d", limit_switch_id_[joint_id]);
            set_mode(pi, limit_switch_id_[joint_id], PI_INPUT);
            set_pull_up_down(pi, limit_switch_id_[joint_id], PI_PUD_DOWN);
            ROS_INFO("Loaded all Parameter for := %s", controller_name.c_str());
        } // end for each joint
        joint_homing_srv_ = nh_.advertiseService("odrive_control/joint_init_to_home", &ODriveHWInterface::joint_init_to_home, this);
        joint_idle_srv_ = nh_.advertiseService("odrive_control/joint_init_to_idle", &ODriveHWInterface::joint_init_to_idle, this); // added by ribin for idle funtion
        pause_robot_srv_ = nh_.advertiseService("odrive_control/pause_joints", &ODriveHWInterface::pause_joints, this);            // added by ribin for pause function
    }

/*
   bool ODriveHWInterface::SetPointRequest(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp)
   {
   long response = 0;
   int joint_id = 0;
   float SetPointValue = 0;
   joint_id = (int)req.joint_id;
   SetPointValue = (float) req.SetPointValue ;
   ROS_INFO("%s, Joint SetPointrequest Recieved for Joint %s : Value %f", joint_names_[joint_id].c_str(),SetPointValue );

   }
*/
// Code segment to create a published
int Joint3JointID = 0 ;
int Joint4JointID = 1 ;
int Joint5JointID = 2 ;
int Joint2JointID = 3 ;

void  ODriveHWInterface::Joint3PositionCallBack(const std_msgs::Float64::ConstPtr& msg)
{
   ROS_INFO(" Joint3PositionCallBack>> data Value : %f",  msg->data);
   //joint_position_command_[Joint3JointID] = msg->data.c_str());
   joint_position_command_[Joint3JointID] = msg->data ;
}

void  ODriveHWInterface::Joint4PositionCallBack(const std_msgs::Float64::ConstPtr& msg)
{
   ROS_INFO(" Joint4PositionCallBack>> data Value : %f",  msg->data);
   joint_position_command_[Joint4JointID] = msg->data;
}

void  ODriveHWInterface::Joint5PositionCallBack(const std_msgs::Float64::ConstPtr& msg)
{
   ROS_INFO(" Joint5PositionCallBack>> data Value : %f",  msg->data);
   joint_position_command_[Joint5JointID] = msg->data;
}
void  ODriveHWInterface::Joint2PositionCallBack(const std_msgs::Float64::ConstPtr& msg)
{
   ROS_INFO(" Joint2PositionCallBack>> data Value : %f",  msg->data);
   joint_position_command_[Joint2JointID] = msg->data;
}

 void chatterCallback(const std_msgs::String::ConstPtr& msg)
 {
   ROS_INFO("I heard: [%s]", msg->data.c_str());
 }

void  ODriveHWInterface::Joint3PositionDirectValueCallBack(double data)
{
   ROS_INFO(" Joint3PositionDirectValueCallBack>> data Value : %f",  data);
   //joint_position_command_[Joint3JointID] = data.c_str());
   joint_position_command_[Joint3JointID] = data ;
}

void  ODriveHWInterface::Joint4PositionDirectValueCallBack(double data)
{
   ROS_INFO(" Joint4PositionDirectValueCallBack>> data Value : %f",  data);
   joint_position_command_[Joint4JointID] = data;
}

void  ODriveHWInterface::Joint5PositionDirectValueCallBack( double data)
{
   ROS_INFO(" Joint5PositionDirectValueCallBack>> data Value : %f",  data);
   joint_position_command_[Joint5JointID] = data;
}
void  ODriveHWInterface::Joint2PositionDirectValueCallBack( double data)
{
   ROS_INFO(" Joint2PositionDirectValueCallBack>> data Value : %f",  data);
   joint_position_command_[Joint2JointID] = data;
}



bool  ODriveHWInterface::PublishJointStateValues()
{

    joint_state.header.stamp = ros::Time::now();
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) 
    {
       std_msgs::Float64 position64 ;
       position64.data = joint_position_[joint_id];
       //TODO : Manohar Sambandam 24FEB2023 , modify this to send a DataPtr so that if in the same node
       //       the publish subscribe will be optimised 
       // Commend in the wiki page
       // When a publisher and subscriber to the same topic both exist inside the same node, 
       // roscpp can skip the serialize/deserialize step (potentially saving a large amount of processing and latency).i
       // It can only do this though if the message is published as a shared_ptr:
    // Commented out the following "joint_states" publisher : 6 MARCH 2023 Manohar Sambandam
       //ODriveJointStatePublisher[joint_id].Publisher.publish(position64  ) ;

       //ROS_INFO("ODriveJointStatePublisher>> Joint: %s  position: %f ) ",\
             (char *)ODriveJointStatePublisher[joint_id].name,\
              joint_position_[joint_id]) ;
       joint_state.position[joint_id] = joint_position_[joint_id];
    }
    // Commented out the following "joint_states" publisher : 6 MARCH 2023 Manohar Sambandam
    joint_pub.publish(joint_state);
    return (true) ;
}

/*  EXPERIMENTAL CODE FOR Callback With message Event
// void callback(const ros::MessageEvent<std_msgs::String const>&);
void ODriveHWInterface::JointPositionCallBack(const ros::MessageEvent<std_msgs::String const>&);
{    

    using BoostCallback = boost::function<void(const ros::MessageEvent<std_msgs::Float64>&>;
    ////subInt32 = node.subscribe<std_msgs::Int32>("Int32", queueSize, BoostCallback(SubscriberCallback<std_msgs::Int32>()));
    subInt32 = node.subscribe< ros::MessageEvent<std_msgs::Int32> >(controller_name_ , queueSize, BoostCallback( ODriveHWInterface::JointPositionCallBack(const ros::MessageEvent<std_msgs::String const>&)) );
    ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);
    ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);

    subInt32 = node.subscribe< ros::MessageEvent<std_msgs::Int32> >(controller_name_ , queueSize, BoostCallback( ODriveHWInterface::JointPositionCallBack(const ros::MessageEvent<std_msgs::String const>&)) );
    subInt32 = node.subscribe< ros::MessageEvent<std_msgs::Int32> >(controller_name_ , queueSize, BoostCallback( ODriveHWInterface::JointPositionCallBack(const ros::MessageEvent<std_msgs::String const>&)) );


void ODriveHWInterface::JointPositionCallBack(const boost::shared_ptr<const  ros::MessageEvent<std_msgs::Float64> message) {
  boost::shared_ptr<geometry_msgs::Pose> pose = message.instantiate<geometry_msgs::Pose>();
  boost::shared_ptr<geometry_msgs::Pose> data  = message.data
   ROS_INFO_STREAM("Publisher: " << event.getPublisherName() 
                    << " header: " << event.getConnectionHeaderPtr());
   typename Msg::ConstPtr msg = event.getConstMessage(

 // The subscriber somewhere in your code...
ros::Subscriber subscriber = node_handle.subscribe<topic_tools::ShapeShifter>("pose_topic", 1, &pose_subscriber);
	const geometry_msgs::PoseStampedConstPtr& msg = event.getMessage();
		proxyPoseStamped *proxyPoseStamped_msg = new proxyPoseStamped();

		// Filling in data for *proxyPoseStamped.idl*
		std::string frame_id = "/";
		frame_id.append(robot_name.c_str());
		frame_id.append(std::string(msg->header.frame_id).c_str());

		proxyPoseStamped_msg->header.frame_id = frame_id.c_str();
		proxyPoseStamped_msg->header.seq = msg->header.seq;
		proxyPoseStamped_msg->header.stamp.nsec = msg->header.stamp.nsec;
		proxyPoseStamped_msg->header.stamp.sec = msg->header.stamp.sec;

        // fill pose/position
		proxyPoseStamped_msg->pose.position.x = msg->pose.position.x;
		proxyPoseStamped_msg->pose.position.y = msg->pose.position.y;
		proxyPoseStamped_msg->pose.position.z = msg->pose.position.z;


// Do stuff...
}
*/


void ODriveHWInterface::JointPositionCallBack(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("JointPositionCallBack>> data Value : %f", msg->data);
    // joint_position_command_[joint_id] = msg->data.c_str());
}

bool ODriveHWInterface::pause_joints(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp)
{
    int joint_id = 0;
    ROS_INFO("%s, Joint PAUSE request Recieved", joint_names_[joint_id].c_str());
    return true;
}

bool ODriveHWInterface::joint_init_to_idle(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp) // added by ribin for initialise joints to idle posiiton
{
    Block_Read_Write_During_Initialization = 1; // setting the flag high so the read and write functions wont be executed
    long response = 0;
    int joint_id = 0;
    joint_id = (int)req.joint_id;
    ROS_INFO("%s, Joint IDLE request Recieved", joint_names_[joint_id].c_str());
    ros::Duration(1).sleep();
    // Commented out to avoid reading before initialization / Bug 05MAR2023 Manohar Sambandam
    // Check the Axis Error
    /*
    if(odrive_serial_interface_[joint_id]->GetError() != 0)
        {
        resp.reason = "Axis error \n";
        ROS_ERROR("%s, Axis Error: %ld", joint_names_[joint_id].c_str(), response);
        return false;
        }
    */
    if (odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_IDLE, STATE_RQST_WAIT) != true)
    {
        resp.reason = "Unable to enter idle state \n";
        ROS_ERROR("%s, Unable to enter idle state", joint_names_[joint_id].c_str());
        return false;
    }
    Block_Read_Write_During_Initialization = 0; // setting the flag to zero so that  the read and write functions can work
    return true;
}

bool ODriveHWInterface::joint_init_to_home(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp)
{
    Block_Read_Write_During_Initialization = 1; // /setting the flag high so the read and write functions wont be executed
    long response = 0;
    int joint_id = 0;
    std_srvs::SetBool srv;
    long target = 0;
    float target_tmp = 0;
    float target_step = 0.05;    // target_step = 0.122
    float target_step_l2 = 3.05; // edited by ribin for moving the l2  faster than other joints

    joint_id = (int)req.joint_id;

    // initialising wiringpi for limit switch gpio

    // TODO//( l5 l3 should be initialised before l4 second)

    ROS_INFO("%s, Initialisation Request Recieved", joint_names_[joint_id].c_str());

    // Check the Axis Error
    /*
    if (odrive_serial_interface_[joint_id]->GetError() != 0)
    {
        resp.reason = "Axis error \n";
        ROS_ERROR("%s, Axis Error: %ld", joint_names_[joint_id].c_str(), response);
        return false;
    }
    */


    // Set the Encoder is_ready to true
    // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_ENCODER_IS_READY_RW, true);

    // added by ribin for checking the limit swithc status before starting the encoder search for reversing the direction
    // joint_homing_switch_in_[joint_id].call(srv);   // commenting for limit switch remove

    // if (srv.response.success == true)
    // int limit_switch_gpio_value = digitalRead(limit_switch_id_[joint_id]);
    int limit_switch_gpio_value = gpio_read(pi, limit_switch_id_[joint_id]);
    ROS_INFO("Value  of Limit Switch =%d", limit_switch_gpio_value);
    if (limit_switch_gpio_value == 1)
    {
        ROS_INFO("Limits switch hit, reversing index search joint id:= %s", joint_names_[joint_id].c_str());
        double reverse_vel = -40;
        double reverse_accel = -20;
        double reverse_ramp_distance = -3.1415927410125732;
        odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_VEL, reverse_vel); // these 3 values are in negative for reversing the index search
        odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_ACCEL, reverse_accel);
        odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_RAMP_DISTANCE, reverse_ramp_distance);
    }

    // Run the state to Index Search and wait for Idle
    if (odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_ENCODER_INDEX_SEARCH, STATE_RQST_NO_WAIT) != true)
    {
        resp.reason = "Unable to enter Encoder Search \n";
        ROS_ERROR("%s, Unable to enter Encoder Search", joint_names_[joint_id].c_str());
        return false;
    }
    ros::Duration(3).sleep();
    ROS_INFO("AXIS_STATE_ENCODER_INDEX_SEARCH finished");
    // Run the state to Closed Loop Control, and not return
    odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT);
    ros::Duration(0.5).sleep();
    if (odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
    {
        resp.reason = "Unable to enter in Closed Loop Control\n";
        ROS_ERROR("%s, Unable to enter in Closed Loop Control", joint_names_[joint_id].c_str());
        return false;
    }
    ROS_INFO("AXIS_STATE_CLOSED_LOOP_CONTROL finished");

    // Check whether index was really found
    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_INDEX_FOUND_R, &response);

    if (response == 0)
    {
        resp.reason = "Unsuccessful Index Search \n";
        ROS_ERROR("%s, Unsuccessful Index Search", joint_names_[joint_id].c_str());
        return false;
    }

    else
    {
        ROS_INFO("%s, Index Search Success", joint_names_[joint_id].c_str());
    }
    // Move the Joint to get hit to Limit Switch for Homing Position
    // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW, (min_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id])/60);
    target = reference_target_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];

    // target_step *= direction_[joint_id];
    while (fabs(target - target_tmp) > 0.0)
    {
        //TODO Disabling doing GetError because it seems to be at error 2MARCH2023
        /*
        long error = odrive_serial_interface_[joint_id]->GetError();
        if (error != 0)
        {
            ROS_ERROR("Error: %ld", error);
            return false;
        }
        */

        if (target < 0 && joint_id != 3)
        {
            target_tmp -= target_step;
            std::cout << target_step << std::endl;
            ROS_INFO("Just for debug :%f", target_tmp);
        }
        // edited by ribin >>we have added condition for checking the initilaisation is for l2 or not, and if l2 we are changing the target_step to target_step_l2
        if (target >= 0 && joint_id != 3)
        {
            target_tmp += target_step;
        }

        if (target < 0 && joint_id == 3)
        {
            target_tmp -= target_step_l2;
            std::cout << target_step << std::endl;
            ROS_INFO("Just for debug :%f", target_tmp);
        }
        if (target >= 0 && joint_id == 3)
        {
            target_tmp += target_step_l2;
            std::cout << target_step << std::endl;
            ROS_INFO("Just for debug :%f", target_tmp);
        }

        // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, target_tmp);
        // modified with trajectory
        int motor_id = axis_id_[joint_id];
        odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, target_tmp);

        sleep(1);

        // joint_homing_switch_in_[joint_id].call(srv); // commenting for limit switch remove
        limit_switch_gpio_value = gpio_read(pi, limit_switch_id_[joint_id]);
        // limit_switch_gpio_value = digitalRead(limit_switch_id_[joint_id]);

        // if(srv.response.success == true)
        if (limit_switch_gpio_value == 1)
            break;
        else
            ROS_INFO("%s: Limit Switch not hit, target: %lf", joint_names_[joint_id].c_str(), target_tmp);
    }
    // Stop the motor
    // odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &zero_offset_[joint_id]);
    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[joint_id]);
    ROS_INFO(" %s: Zero Offset at limit_switch_hit_pos: %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);

    // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
    // modified by trajectory
    int motor_id = axis_id_[joint_id];
    odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, zero_offset_[joint_id]);
    // joint3 id is 0
    if (joint_id == 0)
    {
        // Adding Offset to Joint3 to Initialise Joint4
        zero_offset_[joint_id] += homing_pos_init_seq_[0] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];
        ROS_INFO(" %s : zero_offset_ at homing_pos_init_seq_ %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
    }

    // Adding the homing position to zero_offset_
    else
    {
        zero_offset_[joint_id] += homing_position_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];
        ROS_INFO("%s: Zero offset at homing_position: %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
    }
    // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
    //   int motor_id = axis_id_[joint_id];
    odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, zero_offset_[joint_id]);
    // Joint4 id = 1
    if (joint_id == 1)
    {
        int JOINT3_ID = 0;
        int JOINT3_MOTOR_ID = 0;
        // Move Joint3 to the Joint3 homing_postion instead of the JOINT3 homing_pos_init_seq
        sleep(1);
        zero_offset_[JOINT3_ID] += homing_position_[JOINT3_ID] * transmission_factor_[JOINT3_ID] * encoder_resolution_[JOINT3_ID] * direction_[JOINT3_ID];
        ROS_INFO("%s: Zero offset at homing_position: %lf", joint_names_[JOINT3_ID].c_str(), zero_offset_[JOINT3_ID]);
        // odrive_serial_interface_[0]->WriteProperty(ODRV_POSITION_RW, zero_offset_[0]);
        // modified for trajectory
        // int motor_id = axis_id_[joint_id];
        odrive_serial_interface_[JOINT3_ID]->Write_command_float(ODRV_MOVE_TO_POSE, JOINT3_MOTOR_ID, zero_offset_[JOINT3_ID]);
    }
    //    ROS_INFO("Zero Offset: %ld", zero_offset_[joint_id]);

    // Set the max. velocity to corresponding max. value
    // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW, (max_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id])/60);

    ++No_Of_Joint_initialised_to_home_;
    ROS_INFO("Number Of Joints Initialised :%d", (int)No_Of_Joint_initialised_to_home_);
    /*
      sleep(1.0);
      odrive_serial_interface_[0]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[0]);
      sleep(0.2);
      ROS_INFO(" %s: Zero Offset at Homing_Pos: %lf",joint_names_[0].c_str(),zero_offset_[0]);

      odrive_serial_interface_[1]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[1]);
      sleep(0.2);
      ROS_INFO(" %s: Zero Offset at Homing_Pos: %lf",joint_names_[1].c_str(),zero_offset_[1]);

      odrive_serial_interface_[2]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[2]);
      sleep(0.2);
      ROS_INFO(" %s: Zero Offset at Hooming_Pos: %lf",joint_names_[2].c_str(),zero_offset_[2]);
    */
    /*
    sleep(1);
    //float  zero_offset_home_l3;
    //float  zero_offset_home_l4;
    //float  zero_offset_home_l5;
    ROS_INFO("Three Joints are Initialised ,Reading Encoder Value at Homing Position ");
    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_home_[0]);
    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_home_[1]);
    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_home_[2]);
    //     sleep(1);
    ROS_INFO("%s: Actual Zero offset at homing_position from encoder: %lf",joint_names_[0].c_str(),zero_offset_home_[0]);
    ROS_INFO("%s: Actual Zero offset at homing_position from encoder: %lf",joint_names_[1].c_str(),zero_offset_home_[1]);
    ROS_INFO("%s: Actual Zero offset at homing_position from encoder: %lf",joint_names_[2].c_str(),zero_offset_home_[2]);
    */
    Block_Read_Write_During_Initialization = 0; // setting the flag to 0 so that the read and write can function without any problem
    return true;
    }

void ODriveHWInterface::read(ros::Duration &elapsed_time)
{
    ROS_DEBUG("Inside Read Loop");
    //dynamixel_msgs::JointState joint_state;

    int NoOfJointsToBeInitialised = num_joints_;
    if (height_scan_enable == true) // If Height Scan is not enabled L2 (the vertical scan joint is disabled.
        NoOfJointsToBeInitialised = num_joints_;
    else
        NoOfJointsToBeInitialised = num_joints_ - 1;

    // Run for all the Joints
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    { // the calling flag is used for blocking the execution of read and write during the initialisation service
        if ((No_Of_Joint_initialised_to_home_ == NoOfJointsToBeInitialised) && (Block_Read_Write_During_Initialization == 0))
        {
            // Read current Position
            double joint_current_pos = 0.0;
            ROS_DEBUG("Outer Loop of Reading from :%s which is at joint_id: %d", joint_names_[joint_id].c_str(), (int)joint_id);
            // odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
            //  DONOT READ L2 which is  joint_id =3 && height_scan_enable == False   Feb13 2023 by Mani Radhakrishnan
            bool ConditionToReadWriteJoint = joint_id != 3 || height_scan_enable == true;
            if (ConditionToReadWriteJoint)
            {
                ROS_DEBUG("Reading from :%s which is at joint_id: %d", joint_names_[joint_id].c_str(), (int)joint_id);
                odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &joint_current_pos);
                // ROS_WARN("is this the one being called from odrive_hw_interface");
                ROS_DEBUG("odrive_hw_interface::read() >> Read Finished ");
                joint_current_pos -= zero_offset_[joint_id];
                joint_current_pos *= direction_[joint_id];

                ROS_DEBUG("transmission_factor_[joint_id] : %f , encoder_resolution_[joint_id] %f", (float)transmission_factor_[joint_id], (float)encoder_resolution_[joint_id]);
                ROS_DEBUG("joint_current_pos %f", joint_current_pos);
                ROS_DEBUG("joint_position_[joint_id] %f", joint_position_[joint_id]);

                joint_position_[joint_id] = (joint_current_pos / transmission_factor_[joint_id]) / encoder_resolution_[joint_id];
            }
        }
        ROS_DEBUG("odrive_hw_interface::read()  calling joint_state_publisher for joint_id %d", (int) joint_id);
        //joint_state_pub_[joint_id].publish(joint_state);
        //PublishJointStateValues() ;//ODriveJointStatePublisher();

    }
    ROS_DEBUG("odrive_hw_interface::read()  Finished read Loop");
}

void ODriveHWInterface::write(ros::Duration &elapsed_time)
{   static int counthz = 0 ;
    /// Thisis a debug loop 
    if ( ++counthz == 100 ) {
       ROS_INFO("Inside Write Loop");
       counthz = 0 ;
    }

    int NoOfJointsToBeInitialised = num_joints_;
    if (height_scan_enable == true) // If Height Scan is not enabled L2 (the vertical scan joint is disabled.
        NoOfJointsToBeInitialised = num_joints_;
    else
        NoOfJointsToBeInitialised = num_joints_ - 1;
    // Safety
    // enforceLimits(elapsed_time);

    ROS_DEBUG("ODriveHWInterface::write num_joints : : %d ", (int)num_joints_);
    if ((No_Of_Joint_initialised_to_home_ == NoOfJointsToBeInitialised) && (Block_Read_Write_During_Initialization == 0))
    {
        for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        {
            // Read current Position
            double joint_current_pos = 0.0;
            ROS_DEBUG("ODriveHWInterface:::write() heigh_scan_enable : %s", height_scan_enable ? "True" : "False");
            // odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
            //  DONOT READ L2 which is  joint_id =3 && height_scan_enable == False   Feb13 2023 by Mani Radhakrishnan
            bool ConditionToReadWriteJoint = joint_id != 3 || height_scan_enable == true;
            if (ConditionToReadWriteJoint)
            {
               ROS_DEBUG("ODriveHWInterface:::write() inside ConditionToReadWriteJoint to Write JointID : %d",(int) joint_id );
                // the calling flag is used for blocking the execution of read and write during the initialisation service

                // So that we should not send commands to O-Drive repeatedly
                //<<<<<<<<<<<<<<<<<<<<<<<< edited by ribin to make sure we dont have stol error on 10/03/2020>>>>>>>>>>>>>>

                /*
                   if(joint_position_command_[joint_id] != last_joint_position_command_[joint_id]
                   && (No_Of_Joint_initialised_to_home_ >=3)) //edited ribin for l2 disable
                   */

                // sometime the height scan is not enabled  so we have
                /* COMMENTING TO IDENTIFY THE BUG
                long error = odrive_serial_interface_[joint_id]->GetError();
                if (error != 0)
                {
                    ROS_ERROR("OdriveHWInterface::write joint no %d ERROR : %d", (int)joint_id, (int)error);
                    //ROS_ERROR("OdriveHWInterface::write Shutting Down ROS , calling ros::shutdown()");
                    //system("rosnode kill -a");
                    //ros::shutdown();
                }
                */
                if (joint_position_command_[joint_id] != last_joint_position_command_[joint_id])
                {
                    ROS_INFO("Writing to odrive for joint =: %s", joint_names_[joint_id].c_str());
                    ROS_INFO("ODriveHWInteface::write() %s: Command: %lf, Last Command: %lf", joint_names_[joint_id].c_str(), (float)joint_position_command_[joint_id], (float)last_joint_position_command_[joint_id]);
                    if (joint_id == 3)
                    {
                        // In case of joint_id == 3 ie Link2 used for Vertical scan we enable close loop control and disable it after moving the joint
                        if (odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
                        {
                            // resp.reason = "Unable to enter in Closed Loop Control\n";
                            ROS_ERROR("%s, Unable to enter in Closed Loop Control", joint_names_[joint_id].c_str());
                            // return 0;
                        }
                        long error = odrive_serial_interface_[joint_id]->GetError();
                    }

                    // Todo RIBIN for joint_id==3 we have to make

                    // Check the Joint Limits read from the URDF
                    // TODO put back the check for joint_position limits, temporarily uncommenting this: 25FEB2023 Manohar Sambandam
                    ROS_WARN(" Not Checking for joint Limits for the joint : %s", joint_names_[joint_id].c_str());
                    //if ((joint_position_command_[joint_id] >= joint_position_lower_limits_[joint_id]) && (joint_position_command_[joint_id] <= joint_position_upper_limits_[joint_id]))
                    if (true) {
                        float output_value = 0;
                        // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW,
                        //                                                   (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id])
                        //                                                    + zero_offset_[joint_id]);
                        ROS_INFO(" %s:Zero Offset at Homing_Pos to be added to Joint Value : %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
                        ROS_INFO("joint_position_command_ := %d", (int)joint_position_command_[joint_id]);
                        output_value = (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]) + zero_offset_[joint_id];
                        ROS_INFO("%s: Output Value: %lf", joint_names_[joint_id].c_str(), (float)output_value);

                        // important we are gonna command out this  function, which is responsible for moving the arm
                        //******************** TODO pose set point //
                        //  odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, output_value);
                        // now we are gonna replace it with Trajectory_write
                        // Edited on 28/08/2020 for trajecotry movement/

                        int motor_id = axis_id_[joint_id];
                        ROS_INFO("MOTOR ID %d and joint id %d ", (int)motor_id, (int)joint_id);
                        odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, output_value);
                        ROS_INFO("Serial Interface Write Finished");

                        ROS_INFO("Command Sent to Port: %d, Motor: %d, Error: %ld", odrive_id_[joint_id], axis_id_[joint_id], odrive_serial_interface_[joint_id]->GetError());
                        last_joint_position_command_[joint_id] = joint_position_command_[joint_id];
                    }
                    else
                    {
                        ROS_ERROR("%s: Not Moving, target is beyond the Limits, Joint Command: %lf, whereas Min: %lf, Max: %lf",
                                joint_names_[joint_id].c_str(),
                                joint_position_command_[joint_id],
                                joint_position_lower_limits_[joint_id],
                                joint_position_upper_limits_[joint_id]);
                    }
                }
                ROS_DEBUG("ODriveHWInterface::write() operation loop for joint_id : %d complete ", (int)joint_id);
            }
            //last_joint_position_command_[joint_id] = joint_position_command_[joint_id];
        }
    }

    ROS_DEBUG("odrive_hw_interface::write()  Finished read Loop");
}

void ODriveHWInterface::enforceLimits(ros::Duration &period)
{
    // Enforces position and velocity
    // pos_jnt_sat_interface_.enforceLimits(period);
}
} // namespace odrive_control
