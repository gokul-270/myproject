/*
   Last Updated : 11JAN2024
   Last Compiled : 
   History :
      5JAN2024 : Modified the Read and write function 
	              Read will do a can_read 3 times the no of joints on the hardware (1 for the hearbeat, one for the position estimate + extra one)
				  The last read value will be updated to the current_position of the joint.

				  Write will blindly (TODO : can be optimised ) send the last position command (independent of whether it was written or now) for each of the joing
				  Write will also send a getPositionEstimate odrive can command so that odrive responds with the latest positions
				  If odrive heartbeat supports the position value, (TODO ) then the active getpositionestimate can command can be eliminated
				  TODO : The read and write commands do not go through ROS publish / subcribe 
				  TODO : each of the joints have a local call back function instead of the original publish/subscribe function to avoid ROS overheads
				  TODO : Though it works, there is a lot of code HACKs , which need to be cleaned up

      24JUN2023 - Modifying the Read, Write and Publish functions to reflect the following
	            1. All inputs to odrive_hw_interface will be absolute Arm coordinate system in FLU frame of reference of the robotic arm
				2. Convert the inputs to actual Motor Rotations after accounting for the JointOffset and EncoderOffset values
				3.  JointOffset is the offset of the Zero position of the joint w.r.t the limit switch (used as an absolute reference) in meters or radians
				4.  EncoderOffset is the value (in motor rotations unit) of the limit switch position. 
				The read function reads the absolute position of Motor.
				This Publish function will convert the motor position to FLU system for each joint ( Joint5 which is prismatic in meter, Joint4/3 in radians) 
      15JUN2023 - Adopting odrive_hw_interface for Canbus odriveHWInterface
      28JUN2023 Adding the object controller which is used for Canbus controlling
           TODO:    joint_init_to_home will not work when the limitswitch is ON when it starts
                 the software quits in this conditions
11JAN2024 : Changed the ::read to function to be independent of the initialisation
            Changed the ::write function updated to send a message to the controlleri for the position estimate (independent whether there
            is a write to the joint. This will have to be changed if the controller updates the position in the heartbeat message
 
*/
/*  Author: Saurabh Bansal
            Manohar Sambandam for CAN bus interface
    Desc: To be used as a the canbus interface to the Odrive controller
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
#include <sensor_msgs/JointState.h> //TODO:MR
#include <pigpiod_if2.h>

// Functions for CanBus Interface 
#include <odrive_control/task.hpp>
#include <odrive_control/odrive_can_functions.hpp>
#include <odrive_control/debug_print.hpp>
#include "odrive_tests.cpp"
// JointID used by Yanthra
int Joint3JointID = 0 ;
int Joint4JointID = 1 ;
int Joint5JointID = 2 ;
int Joint2JointID = 3 ;


#include <functional>                                                                               

//void* DUMMY_write_task(void * arg) {
void* readexp(void * args)
        {
            ROS_INFO("Hello in function %s\n", __func__);
        }
void* writeexp(void * args)
        {
            ROS_INFO("Hello in function %s\n", __func__);
        }
class MyClass {                                                                                         
public:
    MyClass(std::function<void(int)> callback) {
        this->callback = callback;
    }   
 
    void doSomething(int value) {
        this->callback(value);
    }   
 
private:
    std::function<void(int)> callback;
};
 
void myCallback(int value) {
    std::cout << "The value is: " << value << std::endl;
}
 
int ixxmain() {
    MyClass myClass(myCallback);
    myClass.doSomething(10);
 
    return 0;
}

void* DUMMY_write_task(void * arg) {
 static int count  = 0 ;
 static float position = 3.0 ;
 printf("%s invocations\n", __func__);
 controller *ControllerPtr ;
 ControllerPtr =  (controller *)arg;
 if (count== 0) PrintThreadID();
 if ( ++count  == 10 ) {
	count = 0 ;
   printf("%s Testing Motor can_id 0 \n", __func__);
   position = position * -1 ;
   //SimpleMotorTest(*ControllerPtr, 0, position );
  }
}

void* DUMMY_read_task(void * arg) {
 static int count  = 0 ;
 static float position = 3.0 ;
 printf("%s invocations\n", __func__);
 controller *ControllerPtr ;
 ControllerPtr =  (controller *)arg;
 if (count== 0) PrintThreadID();
 if ( ++count  == 10 ) {
    count = 0 ;
   printf("%s Testing Motor can_id 1 \n", __func__);
   position = position * -1 ;
   //SimpleMotorTest(*ControllerPtr, 1, position );
 }
}


bool height_scan_enable = false;
int pi;
// added by ribin for seting a flag while initilaisation so that the read and write function wont be called during the initilaisation
int Block_Read_Write_During_Initialization = 1; // Default set it to 1 (until the initialisation is done
namespace odrive_control
{
	ODriveHWInterface::ODriveHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
		:ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
		//: ros_control_boilerplate::GenericHWInterface(nh, urdf_model), CanBusController(writeexp,readexp)
		// controller TestCanBusController(DUMMY_write_task, DUMMY_read_task);
                //controller(void* write_task(void *) ,void* read_task(void *) );
	{
		// Getting the number of Joints, as num_joints_ will be updated in init function
		size_t num_joints = joint_names_.size(); // Code redundant and duplicate but too much to clean it up 15JUN2023
		size_t num_joints_ = joint_names_.size();

		// Initialise the serial interfaces for O-Drives
		// added by ribin on 10/03/2020 for clearing the stol error_code
		nh_.param<bool>("joint2_init/height_scan_enable", height_scan_enable, false);

		//controller *CanBusController = (controller *)arg;
                //TODO:MR:Removed reboot ,Instead move the arm to Homing position and put It In idle
		ROS_INFO("Removed Rebooting odrive ,");
 /*Read the Homing Value from file and move to the position */               
                

                //CanBusController.reboot_odrive(0);
                //CanBusController.reboot_odrive(1);
                //CanBusController.InitialiseAxis(0);
                //CanBusController.InitialiseAxis(1);
	        //SimpleMotorTest(CanBusController, 0, -5 ) ;
	        //SimpleMotorTest(CanBusController, 1, -5 ) ;
		//sleep(1) ;
	        //SimpleMotorTest(CanBusController, 0, 5 ) ;
	        //SimpleMotorTest(CanBusController, 1, 5 ) ;

                //CanBusController.SetReadWriteFunction(DUMMY_write_task, DUMMY_read_task) ;
		// Take objects of class odrive_control, of count 'num_joints'
		CanID.resize(num_joints); // Added CanID for including CANID for every joint
		odrive_id_.resize(num_joints);
		axis_id_.resize(num_joints); // TODO : this is used by the upper level layers (yanthra_move)
		limit_switch_id_.resize(num_joints);
		transmission_factor_.resize(num_joints);
		JointOffSet.resize(num_joints);
		encoder_resolution_.resize(num_joints);
		direction_.resize(num_joints);
		p_gain_.resize(num_joints);
		v_gain_.resize(num_joints);
		v_int_gain_.resize(num_joints);
		zero_offset_.resize(num_joints, 0.0);
		EncoderOffSet.resize(num_joints, 0.0);
		//zero_offset_home_.resize(num_joints, 0.0);
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

	        //ros::Publisher joint_state_pub_[num_joints];
		//ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", num_joints_);
		// joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", num_joints_);
		// ODriveJointStatePublisher.resize(num_joints_);
		// joint_state_.position.resize(num_joints_);
		// joint_state_.name.resize(num_joints_);

		// sensor_msgs::JointState joint_state;
		joint_state.name.resize(num_joints_);
		joint_state.position.resize(num_joints_);
		// joint_state.name[joint_id] = joint_names_[joint_id];
		//  Check whether user has requested for reconfiguration of O-Drives from yaml file
		nh_.param("hardware_interface/odrive_reconfigure_request", odirve_configuration_required_, false);
		std::string controller_name = joint_names_[0] + "_position_controller/";

		nh_.getParam(controller_name + "homing_pos_init_seq_", homing_pos_init_seq_[0]); // added by ribin for initialisation of joint sequance without assistance

		// initalising pigpiod
		ROS_INFO("initalising pigpiod");
		pi = pigpio_start(NULL, NULL);
		if (pi < 0)
		{
			ROS_ERROR("pigpio_start() function did not successful");
			// exit(1) ;
		}
               // Initialise all the odrive connected to the arm
        /* TODO WITH proper CanBusController
                CanBusController.reboot_odrive(0);
                CanBusController.reboot_odrive(1);
	*/
                //std::function<void(int)> callback;
                //CanBusController.SetReadWriteFunction(std::function<void*(void*)> writeexp, std::function<void*(void*)> readexp) ;
                
                //CanBusController.SetReadWriteFunction(writeexp, readexp) ;
                
                //                          void* writeexp(void * args)     MyClass(std::function<void(int)> callback) 
                //void SetReadWriteFunction(void* WriteFunctionPtr(void *), void* rt_read_task(void*)) ;

		ROS_INFO("number of joints_present := %d", (int)num_joints);
		//return ;

		// Initialisation of all Joint Actuators
		for (std::size_t joint_id = 0; joint_id < num_joints; ++joint_id)
		{
			// Read the Joint's parameters
			std::string controller_name = joint_names_[joint_id] + "_position_controller/";
			// std::string controller_name = joint_names_[joint_id] ;

			nh_.getParam(controller_name + "CanID", CanID[joint_id]); // Added 15JUN2023 to include a CAN ID for every joint (motor)
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
			nh_.getParam(controller_name + "JointOffSet",JointOffSet[joint_id]);
			last_joint_position_command_[joint_id] = 0.0; // TODO: Check if it is right value

		    ROS_INFO("Present Joint := %s", controller_name.c_str());
			ROS_INFO(" CanID : %d ", (int)CanID[joint_id]);
			ROS_INFO(" odrive_id : %d ", (int)odrive_id_[joint_id]);
			ROS_INFO("axis_id : %d ", (int)axis_id_[joint_id]);
			ROS_INFO("transmission_factor : %d ", (int)transmission_factor_[joint_id]);
			ROS_INFO("resolution : %d ", (int)encoder_resolution_[joint_id]);
			ROS_INFO("direction : %d ", (int)direction_[joint_id]);
			ROS_INFO("p_gain : %f ", (float)p_gain_[joint_id]);
			ROS_INFO("v_gain : %f ", (float)v_gain_[joint_id]);
			ROS_INFO("v_int_gain : %f ", (float)v_int_gain_[joint_id]);
			ROS_INFO("max_t : %f ", (float)temp_threshold_[joint_id]);
			ROS_INFO("max_e : %f ", (float)error_threshold_[joint_id]);
			ROS_INFO("max_cur : %f ", (float)current_threshold_[joint_id]);
			ROS_INFO("max_vel : %f ", (float)max_velocity_[joint_id]);
			ROS_INFO("min_vel : %f ", (float)min_velocity_[joint_id]);
			// ROS_INFO( "switch : %s",   switch_id_[joint_id]); // Formating is wrong for the string
			ROS_INFO("limit_switch : %d ", (int)limit_switch_id_[joint_id]);
			ROS_INFO("reference_trgt : %f ", (float)reference_target_[joint_id]);
			ROS_INFO("homing_po : %f ", (float)homing_position_[joint_id]);
			last_joint_position_command_[joint_id] = 0.0; // TODO: Check if it is right value

			// Initialise the joint
        		ROS_INFO("%s >> Initialising the CanNode %d , joint_d %d", __func__ ,CanID[joint_id], joint_id);
			int error =  CanBusController.InitialiseAxis( CanID[joint_id]);
			if ( error ) { 
				ROS_ERROR ("%s >> ERROR initialisation of the joint_id %di ErrorCode :: %ld\n", __func__ ,joint_id, error);
				exit(-1) ; 
			}
			ROS_INFO("%s >> *** INITIALISED *** CanNode %d , joint_d %d", __func__ ,CanID[joint_id], joint_id);

			ROS_INFO("MOVING THE JOINT TO HOMING POSITION");
                       // void controller::set_pos_setpoint(uint32_t node_id, float pos_setpoint, float vel_ff,float current_ff)
			//CanBusController.set_pos_setpoint(LocalCanID, joint_homing_value, 0.0, 0.0);

                       /*If Flag is Set to 1 ,then read the value from file otherwise dont read value from file */

                        FILE* inputFile = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_values.txt", "r");
                        FILE* inputFileFlag = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_flag.txt", "r");
                        //char line[256]; // Assuming lines are not longer than 256 characters
                        double joint_homing_values[3];
                        int joint_homing_flag;
                       // Read three values into the joint_homing_values array
                       int fscanf_result= fscanf(inputFileFlag, "%d", &joint_homing_flag); 
                       //if (fscanf(inputFileFlag, "%d", &joint_homing_flag) == 1) 
                       printf("READ FLAG VALUE  %d\n", joint_homing_flag);
                       printf("FSCANF_RESULT  %d\n", fscanf_result);
                      if (fscanf_result== 1 && joint_homing_flag == 1)
                       {
                       //printf("READ FLAG VALUE  %d\n", joint_homing_flag);
                       for (int i = 0; i < 3; i++) 
                       {
                       if (fscanf(inputFile, "%lf", &joint_homing_values[i]) != 1) {
                                 fprintf(stderr, "Error: Unable to read data from the file.\n");
                                 //fclose(inputFile);
                                 //fclose(inputFileFlag);
                                 //return 1;
                          }
                       printf("READ VALUE FOR JOINT ID %d: %lf\n", i, joint_homing_values[i]);
                          }
                          
                       CanBusController.set_pos_setpoint(CanID[joint_id], joint_homing_values[joint_id], 0.0, 0.0);
                          }
                          fclose(inputFile);
                          fclose(inputFileFlag);

			//for (int  joint_id = 0; joint_id < num_joints_; ++joint_id)
                        //for (int joint_id= 0; i < 3; i++) 
                       //{
 /*                               if (fscanf(inputFile, "%lf", &joint_homing_value) == 1) 
                                {
                                ROS_INFO("INIT ME");
                                printf("Read value: %lf\n", joint_homing_value); // Print the read value
                                CanBusController.set_pos_setpoint(CanID[joint_id], joint_homing_value, 0.0, 0.0);
                                }
                                else 
                                {
                                       fprintf(stderr, "Error: Unable to read data from the file.\n");
                                       break; // Break the loop if reading fails
                                }
                      // }
                       if (fgets(line, sizeof(line), inputFile) != NULL) {
                                  if (sscanf(line, "%lf", &joint_homing_value) == 1) {
                                  printf("Read value for joint %d: %lf\n", joint_id, joint_homing_value);
                                  CanBusController.set_pos_setpoint(CanID[joint_id], joint_homing_value, 0.0, 0.0);
                       } else {
                       fprintf(stderr, "Error: Unable to parse data for joint %d from the file.\n", joint_id);
                       }
                       } else {
                        fprintf(stderr, "Error: Unable to read data for joint %d from the file.\n", joint_id);
                      break; // Break the loop if reading fails
                      }
                      fclose(inputFile);
*/

			if (odirve_configuration_required_ == true)
				{
				// TODO: We should be able to Read the motor calibration parameters and write them also
				// Initialise the parameters for this Joint
				/* 
				   CanBusController.set_p_gain(CanID[joint_id], p_gain_[joint_id]);
				   CanBusController.set_v_gain(CanID[joint_id], v_gain_[joint_id]);
				   CanBusController.set_v_int_gain(CanID[joint_id], v_int_gain_[joint_id]);
				   CanBusController.set_current_threshold(CanID[joint_id], current_threshold_[joint_id]);
				   int VelocityLimit = (max_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id]) / 60);
				   CanBusController.set_velocity_limit(CanID[joint_id], VelocityLimit);
				 */
				// TODO :Set Current_limit CanBusController.set_velocity_limit(CanID[joint_id] ,VelocityLimit);
				// TODO :Set Position Error Threshold CanBusController.set_velocity_limit(CanID[joint_id] ,VelocityLimit);
			}
			// Publishing the Joint States messages to be used by yanthra_move
			// Changed from 1000 to 5000 on Aug16 to move more angle of the arm during initialisation
			// joint_state_[joint_id] = nh_.advertise<dynamixel_msgs::JointState>(controller_name + "state", 10);
			// joint_state_.name[joint_id]= controller_name + "/state";

			joint_state.name[joint_id] = joint_names_[joint_id];
			// This is for publishing (ROS) joint states for other tools
			//joint_state_pub_[joint_id] = nh_.advertise<sensor_msgs::JointState>(controller_name + "state", 10); //TODO:MR
			//joint_pub = nh_.advertise<sensor_msgs::JointState>(controller_name + "state", 10); //TODO:MR
			joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

			// Subscribe to joint command here
			// ros::Subscriber Joint2Subscriber,Joint3Subscriber,Joint4Subscriber,Joint5Subscriber;
			// TODO Why Not the following
			// JointSubscriber[joint_id] = nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id]+"_position_controller/command", 1, &ODriveHWInterface::JointPositionCallBack[joint_id], this);
			if (joint_id == Joint3JointID)
			{
				Joint3Subscriber = nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id] + "_position_controller/command", 1, &ODriveHWInterface::Joint3PositionCallBack, this);
				ROS_INFO("Subscribed  Joint3PositionCallBack to joint_id : %d", (int)joint_id);
			}
			if (joint_id == Joint4JointID)
			{
				Joint4Subscriber = nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id] + "_position_controller/command", 1, &ODriveHWInterface::Joint4PositionCallBack, this);

				ROS_INFO("Subscribed  Joint4PositionCallBack to joint_id : %d", (int)joint_id);
			}
			if (joint_id == Joint5JointID)
			{
				Joint5Subscriber = nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id] + "_position_controller/command", 1, &ODriveHWInterface::Joint5PositionCallBack, this);

				ROS_INFO("Subscribed  Joint5PositionCallBack to joint_id : %d", (int)joint_id);
			}
			if (joint_id == Joint2JointID)
			{
				Joint2Subscriber = nh_.subscribe<std_msgs::Float64>(joint_names_[joint_id] + "_position_controller/command", 1, &ODriveHWInterface::Joint2PositionCallBack, this);

				ROS_INFO("Subscribed  Joint2PositionCallBack to joint_id : %d", (int)joint_id);
			}

			// Service Client for Joint Homing Limit Switch
			// TODO: Put a check whether services are available
			joint_homing_switch_in_[joint_id] = nh_.serviceClient<std_srvs::SetBool>(switch_id_[joint_id]);
			{ // TODO : Move this part of the code , which is pin initialisation to a separate section outside of this code.
				// chaneged by ribin on 10/03/2020
				// Changing the sequance of initialisation to l2 first so moved this poriton of the code first
				ROS_INFO("SETTING THE PIN FOR %d", limit_switch_id_[joint_id]);
				set_mode(pi, limit_switch_id_[joint_id], PI_INPUT);
				set_pull_up_down(pi, limit_switch_id_[joint_id], PI_PUD_DOWN);
				ROS_INFO("Loaded all Parameter for := %s", controller_name.c_str());
			}

		} // end for each joint
		joint_homing_srv_ = nh_.advertiseService("odrive_control/joint_init_to_home", &ODriveHWInterface::joint_init_to_home, this);
		joint_idle_srv_ = nh_.advertiseService("odrive_control/joint_init_to_idle", &ODriveHWInterface::joint_init_to_idle, this); // added by ribin for idle funtion
		pause_robot_srv_ = nh_.advertiseService("odrive_control/pause_joints", &ODriveHWInterface::pause_joints, this);            // added by ribin for pause function
               //InitialiseCanInterface(CanBusController, 1) ;
	       //SimpleMotorTest(CanBusController, 1) ;
               //InitialiseCanInterface(CanBusController, 0);
	       //SimpleMotorTest(CanBusController, 0) ;
	       ROS_INFO("%s constructor execution completed\n",__func__);
        }

	void ODriveHWInterface::Joint3PositionCallBack(const std_msgs::Float64::ConstPtr &msg)
	{
		ROS_INFO(" Joint3PositionCallBack>> data Value : %f", msg->data);
		// joint_position_command_[Joint3JointID] = msg->data.c_str());
		joint_position_command_[Joint3JointID] = msg->data;
	}

	void ODriveHWInterface::Joint4PositionCallBack(const std_msgs::Float64::ConstPtr &msg)
	{
		ROS_INFO(" Joint4PositionCallBack>> data Value : %f", msg->data);
		joint_position_command_[Joint4JointID] = msg->data;
	}

	void ODriveHWInterface::Joint5PositionCallBack(const std_msgs::Float64::ConstPtr &msg)
	{
		ROS_INFO(" Joint5PositionCallBack>> data Value : %f", msg->data);
		joint_position_command_[Joint5JointID] = msg->data;
	}
	void ODriveHWInterface::Joint2PositionCallBack(const std_msgs::Float64::ConstPtr &msg)
	{
		ROS_INFO(" Joint2PositionCallBack>> data Value : %f", msg->data);
		joint_position_command_[Joint2JointID] = msg->data;
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
		// This function does a ROS publishing of the joint_states ; 
		//  First it updates the current value of the joint
		//  Then calls the .publish(joint_state)

		joint_state.header.stamp = ros::Time::now();
		for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id) 
		{
			//std_msgs::Float64 position64 ;
			//position64.data = joint_position_[joint_id];
			//TODO : Manohar Sambandam 24FEB2023 , modify this to send a DataPtr so that if in the same node
			//       the publish subscribe will be optimised 
			// Comment in the wiki page
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
	        //ROS_ERROR("%s >> ODriveJointStatePublisher publishing at Time :%ld", __func__,joint_state.header.stamp) ;
                //ROS_INFO("%s Inside Publishing Joint State",__func__);
		joint_pub.publish(joint_state);
		return (true) ;
	}

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

	// added by ribin for initialise joints to idle posiiton
	bool ODriveHWInterface::joint_init_to_idle(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp) 
	{
		Block_Read_Write_During_Initialization = 1; // setting the flag high so the read and write functions wont be executed
		long response = 0;
		int joint_id ;
		joint_id = (int)req.joint_id;
		int LocalCanID = CanID[joint_id] ;
	
		CanBusController.set_axis_requested_state( LocalCanID , AXIS_STATE_IDLE) ;
		Block_Read_Write_During_Initialization = 0; // setting the flag to zero so that  the read and write functions can work
		return true;
	}
#define DEBUGINITTOHOME true
	// The motor will hit a limit switch and then move to a known homing position which is specific to the joint stored for each of the axis 
	// TODO : an absolute encoder in these joints can eliminate the need for the limit switch requirement and the long homing sequence.
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

		// TODO//( l5 l3 should be initialised before l4 second)

		ROS_INFO("$s >>%s, Initialisation Request Recieved",__func__, joint_names_[joint_id].c_str());
		//ROS_ERROR("%s >>%s, Initialisation Not done ", __func__, joint_names_[joint_id].c_str());
		int LocalCanID = CanID[joint_id];		// Set the Encoder is_ready to true

		int limit_switch_gpio_value = gpio_read(pi, limit_switch_id_[joint_id]);
		ROS_INFO("Value  of Limit Switch =%d", limit_switch_gpio_value);
		if (limit_switch_gpio_value == 1)
		{
			ROS_ERROR(" %s NOT HANDLING THe CONDITIONS when LimitSwitch is hit before Encoder Search happens ", __func__);
                        ROS_ERROR (" TODO , the reverse of direction for index search can be achieved by setting the joint in ");
			exit(-1);
			ROS_INFO("Limits switch hit, reversing index search joint id:= %s", joint_names_[joint_id].c_str());
			// These code is TODO , not required is absolute encoder is implemented
			/*
			   double reverse_vel = -40;
			   double reverse_accel = -20;
			   double reverse_ramp_distance = -3.1415927410125732;
			   CanBusController.set_reverse_vel(LocalCanID, reverse_vel);
			   CanBusController.set_reverse_accel(LocalCanID, reverse_accel);
			   CanBusController.index_search_reverse_ramp_distance(LocalCanID, reverse_ramp_distance) ;set_reverse_accel(LocalCanID, reverse_ramp_distance);
			// TODO : Need to test and validate the above code ?
			//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_VEL, reverse_vel); // these 3 values are in negative for reversing the index search
			//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_ACCEL, reverse_accel);
			//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_RAMP_DISTANCE, reverse_ramp_distance);
			 */
		}
		// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_ENCODER_IS_READY_RW, true);
		// added by ribin for checking the limit swithc status before starting the encoder search for reversing the direction
		// joint_homing_switch_in_[joint_id].call(srv);   // commenting for limit switch remove

                // TODO this statement is not required, will cause multiple reset to the arm
                // CanBusController.InitialiseAxis(LocalCanID);

                /* All the following code is achieved by the above call 
		// Check the Axis Error
		long error = CanBusController.GetAxisError(LocalCanID);
		if (error != 0)
		{
			ROS_ERROR(" Joint ID : %s returned Error : %ld ", joint_id, error);
			ROS_ERROR("%s, Unable to enter idle state", joint_names_[joint_id].c_str());
			return false;
		}
           	// Set the AXISSTATE to Index Search and wait for Idle
		//CanBusController.set_axis_requested_state(LocalCanID, AXIS_STATE_ENCODER_INDEX_SEARCH);
		ROS_INFO("%s, Index Search Success", joint_names_[joint_id].c_str());
                long error ;
		error = CanBusController.GetAxisError(LocalCanID);
		if (error != 0)
		{
			ROS_ERROR("%s >> Joint ID : %s returned Error : %ld ", __func__, joint_id, error);
			ROS_ERROR("%s >> %s, Unable to enter ENCODER SEARCH State ", __func__, joint_names_[joint_id].c_str());
			return false;
		}
		ROS_INFO("AXIS_STATE_ENCODER_INDEX_SEARCH finished");

		// Run the AXIS to Closed Loop Control
		CanBusController.set_axis_requested_state(LocalCanID, AXIS_STATE_CLOSED_LOOP_CONTROL);

		error = CanBusController.GetAxisError(LocalCanID);
		if (error != 0)
		{
			ROS_ERROR("%s >> Joint ID : %s returned Error : %ld ", __func__, joint_id, error);
			ROS_ERROR("%s >> %s, Unable to enter Closed Loop State ", __func__, joint_names_[joint_id].c_str());
			return false;
		}
		ROS_INFO("AXIS_STATE_CLOSED_LOOP_CONTROL finished");
               */


		// Move the Joint to get hit to Limit Switch for Homing Position
		// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW, (min_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id])/60);
		target = reference_target_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];

		// target_step *= direction_[joint_id];
#define DEBUGWITHOUTLIMITSWITCH  false
                if (DEBUGWITHOUTLIMITSWITCH) { 
                ROS_ERROR ("%s this is workaround code without LimitSwitch \n",__func__) ;
                target_tmp = target ;
                }

                FILE* inputFileFlag = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_flag.txt", "r");
                int joint_homing_flag;
                // Read three values into the joint_homing_values array
                int fscanf_result= fscanf(inputFileFlag, "%d", &joint_homing_flag); 
                printf("READ FLAG VALUE  %d\n", joint_homing_flag);
                printf("FSCANF_RESULT  %d\n", fscanf_result);
               if (fscanf_result== 1 && joint_homing_flag == 0)
               {
                while (fabs(target - target_tmp) > 0.0)
		{
			// TODO Disabling doing GetError because it seems to be at error 2MARCH2023
			/*
			   long error = odrive_serial_interface_[joint_id]->GetError();
			   if (error != 0)
			   {
			   ROS_ERROR("Error: %ld", error);
			   return false;
			   }
			 */

		        /* TODO WITH proper CanBusController
			long error = CanBusController.GetAxisError(LocalCanID);
			if (error != 0)
			{
				ROS_ERROR("Axis Error: %ld", error);
				return false;
			}
			*/
			// TODO : Refactor this code
			// TODO : Refactor this code
			// TODO : Refactor this code
                        printf("GOING FOR LIMIT SWITCH SEARCH  %d\n", fscanf_result);
			if (target < 0 && joint_id != Joint2JointID)
			{
				target_tmp -= target_step;
				//std::cout << target_step << std::endl;
				ROS_INFO("Just for debug :%f", target_tmp);
			}
			// edited by ribin >>we have added condition for checking the initilaisation is for l2 or not, and if l2 we are changing the target_step to target_step_l2
			if (target >= 0 && joint_id != Joint2JointID)
			{
				target_tmp += target_step;
			}

			if (target < 0 && joint_id == Joint2JointID)
			{
				target_tmp -= target_step_l2;
				//std::cout << target_step << std::endl;
				ROS_INFO("Just for debug :%f", target_tmp);
			}
			if (target >= 0 && joint_id == Joint2JointID)
			{
				target_tmp += target_step_l2;
				//std::cout << target_step << std::endl;
				ROS_INFO("Just for debug :%f", target_tmp);
			}

			// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, target_tmp);
			// modified with trajectory
			int motor_id = axis_id_[joint_id];
			// odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, target_tmp);
			CanBusController.set_pos_setpoint(LocalCanID, target_tmp, 0.0, 0.0);
			// TODO : The move_pos is faster than pos_setpoint due to the extra computation required
			// CanBusController.move_pos(LocalCanID , target_tmp) ;
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
		// odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[joint_id]);
		// EncoderOffSet is the OffSet value of the Encoder Zero from Limit Switch
		zero_offset_[joint_id] = CanBusController.GetPositionEstimate(LocalCanID);
                EncoderOffSet[joint_id] =zero_offset_[joint_id];
		CanBusController.set_pos_setpoint(CanID[joint_id], zero_offset_[joint_id], 0.0, 0.0);
        if (DEBUGINITTOHOME) {		long ReadJointOffSet ;
            for (int i = 0 ; i < 2 ; i++) {
                zero_offset_[joint_id] = CanBusController.GetPositionEstimate(LocalCanID);
                //ReadJointOffSet = CanBusController.GetPositionEstimate(LocalCanID); 
                ROS_INFO("%s:Reading without failure iteration %d value = %lf\n", __func__, i, zero_offset_[joint_id]) ;
            }
        }
		if (DEBUGINITTOHOME) ROS_INFO(" %s: Zero Offset at limit_switch_hit_pos: %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);

		// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
		// modified by trajectory
		int motor_id = axis_id_[joint_id];
		// odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, zero_offset_[joint_id]);
		//  joint3 id is 0 JointJoint3JointID3
		if (joint_id == Joint3JointID)
		{
			// Adding Offset to Joint3 to Initialise Joint4 to HOMING_POS_INIT_SEQ a value which is a temporary value to position the link3 in a position
			// which does not hinder the movement of the arm in the cotton field conditions.
			zero_offset_[joint_id] += homing_pos_init_seq_[Joint3JointID] * transmission_factor_[Joint3JointID] * encoder_resolution_[Joint3JointID] * direction_[Joint3JointID];
			if (DEBUGINITTOHOME) ROS_INFO(" %s : zero_offset_ at homing_pos_init_seq_ %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
		}

		// Adding the homing position to zero_offset_
		else
		{
			zero_offset_[joint_id] += homing_position_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];
			if (DEBUGINITTOHOME) ROS_INFO("%s: Zero offset at homing_position: %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
		}
		// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
		//   int motor_id = axis_id_[joint_id];
		// odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, zero_offset_[joint_id]);
		CanBusController.set_pos_setpoint(CanID[joint_id], zero_offset_[joint_id], 0.0, 0.0);
		// Joint4 id = 1 Joint4JointID
		if (joint_id == Joint4JointID)
		{
			int JOINT3_ID = Joint3JointID;
			int JOINT3_MOTOR_ID = 0;
			// Move Joint3 to the Joint3 homing_postion instead of the JOINT3 homing_pos_init_seq
			sleep(1);
			zero_offset_[JOINT3_ID] += homing_position_[JOINT3_ID] * transmission_factor_[JOINT3_ID] * encoder_resolution_[JOINT3_ID] * direction_[JOINT3_ID];
			if (DEBUGINITTOHOME) ROS_INFO("%s: Zero offset at homing_position: %lf", joint_names_[JOINT3_ID].c_str(), zero_offset_[JOINT3_ID]);
			// odrive_serial_interface_[0]->WriteProperty(ODRV_POSITION_RW, zero_offset_[0]);
			// modified for trajectory
			// int motor_id = axis_id_[joint_id];
			// odrive_serial_interface_[JOINT3_ID]->Write_command_float(ODRV_MOVE_TO_POSE, JOINT3_MOTOR_ID, zero_offset_[JOINT3_ID]);
			CanBusController.set_pos_setpoint(CanID[Joint3JointID], zero_offset_[JOINT3_ID], 0.0, 0.0);
		}
                fclose(inputFileFlag);
                ROS_INFO("%s: Tracking Zero offset at homing_position from encoder of Joint3: %lf",joint_names_[0].c_str(),zero_offset_[0]);
               ++No_Of_Joint_initialised_to_home_ ;
               ROS_INFO("No_Of_Joint_initialised_to_home_ = %d" ,No_Of_Joint_initialised_to_home_);
              //Reading the homing Value 
              int NoOfJointsToBeInitialised = num_joints_;
              if (No_Of_Joint_initialised_to_home_ == 3){
                FILE* EncoderOffsetFile = fopen("/home/ubuntu/pragati/src/odrive_control/config/encoder_offset_values.txt", "w");
                ROS_INFO("Writing Encoder Offset Values");
              // Append the values of the first three joints to the file
               for (int i = 0; i < 3 ; i++) 
                {
                fprintf(EncoderOffsetFile, "%lf\n",EncoderOffSet[i]);
                ROS_INFO("Wrote the encoder offset values to file");
                }
                fclose(EncoderOffsetFile);
                }

              if (No_Of_Joint_initialised_to_home_ == 3){
                      sleep(2);//Waiting for Joint3 to  Reach the Homing Position before reading the Value
                  for (int i = 0 ; i < num_joints_ ; i++ ) {
                     zero_offset_[i] = CanBusController.GetPositionEstimate(CanID[i]);
                ROS_INFO("%s: Tracking Zero offset at homing_position from encoder of Joint3: %lf",joint_names_[0].c_str(),zero_offset_[0]);

                      ROS_INFO("I RED VALUE ,AFTER THREE JOINTS INITIALISED");
            ROS_INFO("Three Joints are Initialised ,Reading Encoder Value at Homing Position ");
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[0].c_str(),zero_offset_[0]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[1].c_str(),zero_offset_[1]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[2].c_str(),zero_offset_[2]);
               }

        if (DEBUGINITTOHOME) {
            ROS_INFO("Number Of Joints Initialised :%d", (int)No_Of_Joint_initialised_to_home_);

            sleep(1);
            ROS_INFO("Three Joints are Initialised ,Reading Encoder Value at Homing Position ");
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[0].c_str(),zero_offset_[0]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[1].c_str(),zero_offset_[1]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[2].c_str(),zero_offset_[2]);
            double ReadJointOffSet ;
            for (int i = 0 ; i < 2 ; i++) {
                ReadJointOffSet = CanBusController.GetPositionEstimate(LocalCanID); 
                ROS_INFO("%s: Read  Zero offset at homing_position for jointId : %d CanID %d  ReadValue : %lf",\
                  __func__, joint_id, LocalCanID,ReadJointOffSet) ;
            }
            }
        FILE* outputFile = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_values.txt", "w");
        FILE* outputFileFlag = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_flag.txt", "w");
        ROS_INFO("I AM JUST HERE");
        // Append the values of the first three joints to the file
        int  homing_flag=1;
        fprintf(outputFileFlag, "%d\n", homing_flag);
        fclose(outputFileFlag);
        for (int i = 0; i < 3 ; i++) 
        {
            fprintf(outputFile, "%lf\n",zero_offset_[i]);
            ROS_INFO("Wrote joint homing values to file");
            ROS_INFO("Setting the Flag to Read from File");
        }
       fclose(outputFile);
       }
       }
         else{

                         ROS_INFO("In ELSE CONDITION AVOIDING LIMIT SWITCH SEARCH");
                         //      EncoderOffSet[joint_id] =zero_offset_[joint_id];
	                 double EncoderOffSetValues[3];
                         FILE* EncoderOffsetFile = fopen("/home/ubuntu/pragati/src/odrive_control/config/encoder_offset_values.txt", "r");
                         ROS_INFO("Writing Encoder Offset Values");
                          // Append the values of the first three joints to the file
                          for (int i = 0; i < 3 ; i++) 
                          {
                         if( fscanf(EncoderOffsetFile, "%lf",&EncoderOffSetValues[i]) == 1);
                              {
                               //ROS_INFO("Wrote the encoder offset values to file");
                               EncoderOffSet[i]= EncoderOffSetValues[i];
                               ROS_INFO("ENCODER OFFSET VALUE READ FROM FILE = %lf" ,EncoderOffSet[i]);
                              //EncoderOffSet[joint_id] =zero_offset_[joint_id];
 
                           }
                           }
                           fclose(EncoderOffsetFile);

                         ROS_INFO("No_Of_Joint_initialised_to_home_ = %d" ,No_Of_Joint_initialised_to_home_);
                         No_Of_Joint_initialised_to_home_ =3;
	                 double joint_homing_values[3];
                         FILE* inputFile = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_values.txt", "r");
			 for (int i = 0; i < 3; i++) 
                           {
                       if (fscanf(inputFile, "%lf", &joint_homing_values[i]) == 1) 
					   {
                                 //fprintf(stderr, "Error: Unable to read data from the file.\n");
                                 zero_offset_[i] = joint_homing_values[i];
	                             ROS_INFO("HOMING POS VALUE WITHOUT LIMIT SWITCH SEARCH_ = %lf" ,zero_offset_[i]);
								 //fclose(inputFile);

                                 //fclose(inputFileFlag);
                                 //return 1;
                            }
                            }
			    fclose(inputFile);
        if (DEBUGINITTOHOME) {
            ROS_INFO("Number Of Joints Initialised :%d", (int)No_Of_Joint_initialised_to_home_);

            sleep(1);
            ROS_INFO("Three Joints are Initialised ,Reading Encoder Value at Homing Position ");
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[0].c_str(),zero_offset_[0]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[1].c_str(),zero_offset_[1]);
            ROS_INFO("%s: Stored Zero offset at homing_position from encoder: %lf",joint_names_[2].c_str(),zero_offset_[2]);
            double ReadJointOffSet ;
            for (int i = 0 ; i < 2 ; i++) {
                ReadJointOffSet = CanBusController.GetPositionEstimate(LocalCanID); 
                ROS_INFO("%s: Read  Zero offset at homing_position for jointId : %d CanID %d  ReadValue : %lf",\
                  __func__, joint_id, LocalCanID,ReadJointOffSet) ;
            }

			   }
      } 
        Block_Read_Write_During_Initialization = 0; // setting the flag to 0 so that the read and write can function without any problem
	return true;
	}

#define ODriveREAD_DEBUG  false
	void ODriveHWInterface::read(ros::Duration &elapsed_time)
	{
		// This method will read the joint states periodically and update the joint position values for each of the joint
		//if( ODriveREAD_DEBUG ) ROS_INFO("%s >> Inside Read Loop",__func__);
		//dynamixel_msgs::JointState joint_state;
		sensor_msgs::JointState joint_state; //TODO:MR

		int NoOfJointsToBeInitialised = num_joints_;
		if (height_scan_enable == true) // If Height Scan is not enabled L2 (the vertical scan joint is disabled.
			NoOfJointsToBeInitialised = num_joints_;
		else
			NoOfJointsToBeInitialised = num_joints_ - 1;

		// Run for all the Joints
		//for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        
        // Just reading the actual joint from canbus reads
        
		for (int joint_id = 0; joint_id < (num_joints_ *2) ; ++joint_id) {
          if (CanBusController.can_read())
            CanBusController.ProcessData(); // TODO Check whether ProcessData is also reading Encoder Estimate
          //else break ;
        }

		for (int joint_id = 0; joint_id < num_joints_; ++joint_id)
		{ // the calling flag is used for blocking the execution of read and write during the initialisation service
			// TODO :  these checks can be removed for CANBUS interface unlike serial interface
            // Commented the following on 11JAN2023. We can read unconditional in caninterface
			//if ((No_Of_Joint_initialised_to_home_ == NoOfJointsToBeInitialised) && (Block_Read_Write_During_Initialization == 0))
			{
				// Read current Position
				double joint_current_pos = 0.0;
				if( ODriveREAD_DEBUG ) ROS_INFO("Outer Loop of Reading from :%s which is at joint_id: %d", joint_names_[joint_id].c_str(), (int)joint_id);
				// odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
				//  DONOT READ L2 which is  joint_id =3 && height_scan_enable == False   Feb13 2023 by Mani Radhakrishnan
				bool ConditionToReadWriteJoint = (joint_id != Joint2JointID) || (height_scan_enable == true);
				if (ConditionToReadWriteJoint)
				{
					if( ODriveREAD_DEBUG ) ROS_INFO("Reading from :%s which is at joint_id: %d", joint_names_[joint_id].c_str(), (int)joint_id);
					//odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &joint_current_pos);
					double joint_current_position ;
                         		//zero_offset_[joint_id] = EncoderOffSet[joint_id] = CanBusController.GetPositionEstimate(LocalCanID);
                         		//zero_offset_home_[joint_id] = CanBusController.GetPositionEstimate(LocalCanID);
					joint_current_position = CanBusController.GetPositionEstimate( CanID[joint_id]); 
					//joint_current_position = CanBusController.GetPositionEstimate( LocalCanID); 
					// ROS_WARN("is this the one being called from odrive_hw_interface");

					//if( ODriveREAD_DEBUG ) ROS_INFO("%s >> transmission_factor_[joint_id] : %f , encoder_resolution_[joint_id] %f", __func__, (float)transmission_factor_[joint_id], (float)encoder_resolution_[joint_id]);
					//if( ODriveREAD_DEBUG ) ROS_INFO("%s >> joint_current_position %f",  __func__,joint_current_position);
					//if( ODriveREAD_DEBUG ) ROS_INFO("%s >> joint_position_[joint_id] %f",  __func__,joint_position_[joint_id]);

					//joint_position_[joint_id] = (joint_current_position / transmission_factor_[joint_id]) / encoder_resolution_[joint_id];
					/*
							float MotorRotationFromJointZero = joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]; 
							float MotorRotationFromLimitSwitch = MotorRotationFromJointZero + (JointOffSet[joint_id]  * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]); 
						    float MotorRotationFromMotorZero = MotorRotationFromLimitSwitch - EncoderOffSet[joint_id] ; // EncoderOffSet will be negative value 
							output_value = MotorRotationFromMotorZero ;	
					*/
					double ReadMotorRotationFromMotorZero =  joint_current_position  ; 
					double ReadMotorRotationFromLimitSwitch = ReadMotorRotationFromMotorZero - EncoderOffSet[joint_id] ; //EncoderOffset  should be in Rotations 
					double ReadJointPositionFromLimitSwitch = (ReadMotorRotationFromLimitSwitch / transmission_factor_[joint_id] ) / encoder_resolution_[joint_id] * direction_[joint_id]; 
					double ReadJointPositionFromJointZero = ReadJointPositionFromLimitSwitch - JointOffSet[joint_id];
					joint_position_[joint_id] = ReadJointPositionFromJointZero  ;
					if( ODriveREAD_DEBUG ) { 
				            ROS_INFO(" %s JointID %d  JointCurrentPosition : %lf ",__func__, joint_id, joint_current_position) ;
				            ROS_INFO(" %s JointID %d  ZEROOFFSET OF JOINT: %lf ",__func__, joint_id, zero_offset_[joint_id]) ;
				            ROS_INFO(" %s JointID %d  JointOffSet OF JOINT: %lf EncoderOffSet %lf ",__func__, joint_id,JointOffSet[joint_id], EncoderOffSet[joint_id]);
				            ROS_INFO(" %s JointID %d  ReadMotorRotationFromLimitSwitch OF JOINT: %lf ",__func__, joint_id,ReadMotorRotationFromLimitSwitch );
				            ROS_INFO(" %s JointID %d  ReadJointPositionFromLimitSwitch OF JOINT: %lf ",__func__, joint_id,ReadJointPositionFromLimitSwitch );
				            ROS_INFO(" %s JointID %d ReadJointPositionFromJointZero  OF JOINT: %lf ",__func__, joint_id,ReadJointPositionFromJointZero  );
				            //ROS_INFO(" %s JointID %d  ZEROOFFSET OF HOME POSITION: %lf ",__func__, joint_id, zero_offset_home_[joint_id]) ;
					    joint_current_position -= zero_offset_[joint_id];
					    joint_current_position *= direction_[joint_id];
					    double oldJointPosition = (joint_current_position / transmission_factor_[joint_id]) / encoder_resolution_[joint_id];
				            ROS_INFO(" %s JointID %d  JointPosition New : %lf , OldValue %lf ",__func__, joint_id, ReadJointPositionFromJointZero , oldJointPosition ) ;
					} 
				}
			}
		}
        // PublishJointStateValues();
		//if( ODriveREAD_DEBUG ) ROS_INFO("%s >> Finished read Loop", __func__ );
	}

#define DEBUGWRITE false
	// This call is made ONLY for setting the joint position 
	void  ODriveHWInterface::write(ros::Duration &elapsed_time)
	{   static int counthz = 0 ;
	    static int Testposition = 3 ;
	    int NoOfJointsToBeInitialised = num_joints_;
		/// Thisis a debug loop 
		if (DEBUGWRITE) {
            if ( ++counthz == 100 ) {
			ROS_INFO("Inside 100 th ++  Write Loop");
			counthz = 0 ;
			Testposition  = Testposition * -1 ;
            } 
        }


		{
			// TODO , this section of the code can be done in the initialisation of ODRIVE
			// TODO , this section of the code can be done in the initialisation of ODRIVE
			// TODO , this section of the code can be done in the initialisation of ODRIVE
			if (height_scan_enable == true) // If Height Scan is not enabled L2 (the vertical scan joint is disabled.
				NoOfJointsToBeInitialised = num_joints_;
			else
				NoOfJointsToBeInitialised = num_joints_ - 1;
		}

		// Safety
		// enforceLimits(elapsed_time);

		//ROS_INFO("%s >> ODriveHWInterface::write num_joints : : %d ",__func__, (int)num_joints_);
		// TODO :  these checks can be removed for CANBUS interface unlike serial interface
		if ((No_Of_Joint_initialised_to_home_ == NoOfJointsToBeInitialised) && (Block_Read_Write_During_Initialization == 0))
		{
			for (int  joint_id = 0; joint_id < num_joints_; ++joint_id)
            {
                // Read current Position
                double joint_current_pos = 0.0;
                //ROS_INFO("ODriveHWInterface:::write() heigh_scan_enable : %s", height_scan_enable ? "True" : "False");
                // odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
                //  DONOT READ L2 which is  joint_id =3 && height_scan_enable == False   Feb13 2023 by Mani Radhakrishnan
                bool ConditionToReadWriteJoint = ((joint_id != Joint2JointID ) || (height_scan_enable == true)) ;
                if (ConditionToReadWriteJoint)
                {
                    //if (DEBUGWRITE) ROS_INFO("ODriveHWInterface:::write() inside ConditionToReadWriteJoint to Write JointID : %d",(int) joint_id );
                    if (joint_position_command_[joint_id] != last_joint_position_command_[joint_id])
                    {
                        if (DEBUGWRITE) ROS_INFO("%s Writing to odrive for joint =: %s",__func__, joint_names_[joint_id].c_str());
                        if (DEBUGWRITE) ROS_INFO("%s  %s: Command: %lf, Last Command: %lf", joint_names_[joint_id].c_str(), (float)joint_position_command_[joint_id], (float)last_joint_position_command_[joint_id]);
                        //if (joint_id == 3), this is Joint3 with Joint3JointID 
                        if (joint_id == Joint2JointID  )
                        {
                            // In case of joint_id == 3 ie Link2 used for Vertical scan we enable close loop control and disable it after moving the joint
                            //if (odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
                            CanBusController.set_axis_requested_state(CanID[Joint3JointID],AXIS_STATE_CLOSED_LOOP_CONTROL );
                        }

                        // Todo RIBIN for joint_id==3 we have to make

                        // Check the Joint Limits read from the URDF
                        // TODO put back the check for joint_position limits, temporarily uncommenting this: 25FEB2023 Manohar Sambandam
                        if (DEBUGWRITE) ROS_WARN(" Not Checking for joint Limits for the joint : %s", joint_names_[joint_id].c_str());
                       /* if (DEBUGWRITE) ROS_INFO("%s %s Joint Command Value : %lf, whereas Min: %lf, Max: %lf", __func__,
                                joint_names_[joint_id].c_str(),
                                joint_position_command_[joint_id],
                                joint_position_lower_limits_[joint_id],
                                joint_position_upper_limits_[joint_id]);
                       */
                        // TODO : Checking for the joint physical limits. This should be done in the motor controller and not in the software here.
                        if ((joint_position_command_[joint_id] >= joint_position_lower_limits_[joint_id]) && (joint_position_command_[joint_id] <= joint_position_upper_limits_[joint_id])) 
                        {
                            double output_value = 0;
                            // odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW,
                            //                                                   (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id])
                            //                                                    + zero_offset_[joint_id]);
                            /*
                               Computation for For Joint5 : (similar for Joint4 and Joint3 )
                               For a given “X” value in meters the FLU system
                               (A) MotorRotationFromJointZero (No of Rotation required to move from Joint5’s zero position) =  X * TransmissionFactor
                               (B)  (No Of Rotation Required to Move from LimitSwitch to Joint5’s zero position) 
                               = JointOffset (of Joint5) * TransmissionFactor
                               (C) MotorRotationFromLimitSwitch ( No Of Rotation Required to Move from Limit Switch to X) = A + B
                               (D) MotorRotationFromMotorZero (Not Of Rotation Required to Move from Encoder’s ZeroPosition to X) = A + B + C
                               */
                            double  MotorRotationFromJointZero = joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]; 
                            //TODO MotorRotationFromLimitSwitch is a one time computation after intialisation. Can be fixed instead of computing every cycle
                            double  MotorRotationFromLimitSwitch = MotorRotationFromJointZero + (JointOffSet[joint_id]  * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]); 
                            double  MotorRotationFromMotorZero = MotorRotationFromLimitSwitch + EncoderOffSet[joint_id] ; // EncoderOffSet will be negative value 
                            output_value = MotorRotationFromMotorZero ;	
                            //output_value = (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]) + zero_offset_[joint_id];
                          /*
                           if (DEBUGWRITE) {
                                double old_output_value = (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]) + zero_offset_[joint_id];
                                ROS_INFO(" %s:Zero Offset at Homing_Pos to be added to Joint Value : %lf", joint_names_[joint_id].c_str(), zero_offset_[joint_id]);
                                ROS_INFO("joint_position_command_ := %lf", (double )joint_position_command_[joint_id]);
                                ROS_INFO( "%s  MotorRotationFromJointZero %lf , MotorRotationFromLimitSwitch %lf ", __func__, MotorRotationFromJointZero  , MotorRotationFromLimitSwitch )  ;
                                ROS_INFO( "%s  MotorRotationFromMotorZero %lf ", __func__, MotorRotationFromJointZero  , MotorRotationFromLimitSwitch )  ;
                                ROS_INFO( "%s output_value from New : %lf Old : %lf ", __func__, output_value, old_output_value ) ;
                            }
                          */
                            //if (DEBUGWRITE) ROS_INFO("%s: Output Value: %lf", joint_names_[joint_id].c_str(), output_value);

                            // important we are gonna command out this  function, which is responsible for moving the arm
                            //******************** TODO pose set point //
                            //  odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, output_value);
                            // now we are gonna replace it with Trajectory_write
                            // Edited on 28/08/2020 for trajecotry movement/

                            int motor_id = axis_id_[joint_id];
                            //if (DEBUGWRITE) ROS_INFO("MOTOR ID %d and joint id %d ", (int)motor_id, (int)joint_id);
                            //odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE, motor_id, output_value);
                            CanBusController.set_pos_setpoint( CanID[joint_id], output_value,0.0, 0.0) ;
							double encoder_value =  CanBusController.GetPositionEstimate( CanID[joint_id]);
							ROS_ERROR(" can_id %d encoder_value for the homing pos : %lf ", CanID[joint_id], encoder_value); 
                            if (DEBUGWRITE) ROS_INFO("%s value %lf  CanNodeID  %d, joint_id : %d MotorId : %d\n",__func__ , output_value,  CanID[joint_id], joint_id, motor_id);
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
                        //if (DEBUGWRITE) ROS_INFO("ODriveHWInterface::write() operation loop for joint_id : %d complete ", (int)joint_id);
                    }
                }
            }
		//if (DEBUGWRITE) ROS_INFO(" %s >> odrive_hw_interface::write()  Finished write Loop",__func__);

	   for (int  joint_id = 0; joint_id < num_joints_; ++joint_id)
          CanBusController.RequestMsg.get_encoder_estimate(joint_id);
	}
	void ODriveHWInterface::enforceLimits(ros::Duration &period)
	{
		// Enforces position and velocity
		// pos_jnt_sat_interface_.enforceLimits(period);
	}
} // namespace odrive_control




