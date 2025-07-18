
/*  Author: Saurabh Bansal
Desc:   O-Drive Hardware control, to be used with ros-control
*/

#include <odrive_control/generic_hw_interface.h>
#include <odrive_control/odrive_hw_interface.h>
#include <dynamixel_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <unistd.h>
#include <wiringPi.h>
#include <pigpiod_if2.h>


bool height_scan_enable;
int pi;
int Block_Read_Write_During_Initialization = 0; //added by ribin for seting a flag while initilaisation so that the read and write function wont be called during the initilaisation
namespace odrive_control
{
	ODriveHWInterface::ODriveHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
		: ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
	{
		// Getting the number of Joints, as num_joints_ will be updated in init function
		size_t num_joints = joint_names_.size();

		// Initialise the serial interfaces for O-Drives
		nh_.param<int>("/hardware_interface/baud", baud_rate_, 115200);
		nh_.getParam("/hardware_interface/odrive_0", odrive_serial_number_[0]);
		nh_.getParam("/hardware_interface/odrive_1", odrive_serial_number_[1]);
		//nh_.param<std::string>("/hardware_interface/odrive_1", odrive_serial_number_[0]);
		//nh_.param<std::string>("/hardware_interface/odrive_1", odrive_serial_number_[1]);
		ROS_INFO("Odrive serial number 0 = %s",odrive_serial_number_[0].c_str());
		ROS_INFO("Odrive serial number 1 = %s",odrive_serial_number_[1].c_str());
		//added by ribin on 10/03/2020 for clearing the stol error_code
		nh_.param<bool>("joint2_init/height_scan_enable",height_scan_enable,true);
	
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
		joint_state_pub_.resize(num_joints);
		joint_homing_switch_in_.resize(num_joints);
		No_Of_Joint_initialised_to_home_ = 0;
		homing_pos_init_seq_.resize(num_joints);

		// Check whether user has requested for reconfiguration of O-Drives from yaml file
		nh_.param("hardware_interface/odrive_reconfigure_request", odirve_configuration_required_, false);
		std::string controller_names = joint_names_[0] + "_position_controller/";

		nh_.getParam(controller_names + "homing_pos_init_seq_",homing_pos_init_seq_[0]); //added by ribin for initialisation of joint sequance without assistance

		// Initialisation of all Joint Actuators
		for(std::size_t joint_id = 0; joint_id <num_joints; ++joint_id)
		{
			// Read the Joint's parameters

			std::string controller_name = joint_names_[joint_id] + "_position_controller/";
			ROS_INFO("number of joints_present := %s", controller_name.c_str());

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
			last_joint_position_command_[joint_id] = 0.0;       // TODO: Check if it is right value

			// Initialise the O-Drive Serial Interface
			odrive_serial_interface_[joint_id] = new ODriveSerial(*odrive_serial_port_[odrive_id_[joint_id]],
					axis_id_[joint_id]);

			// If O-Drive configuration is required from configuration .ymal file
			if(odirve_configuration_required_ == true)
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
			joint_state_pub_[joint_id] = nh_.advertise<dynamixel_msgs::JointState>(controller_name + "state", 1000);//Changed from 1000 to 5000 on Aug16 to move more angle of the arm during initialisation

			// Service Client for Joint Homing Limit Switch
			// TODO: Put a check whether services are available
			joint_homing_switch_in_[joint_id] = nh_.serviceClient<std_srvs::SetBool>(switch_id_[joint_id]);

			// Check if there is some Axis Error
			long response = odrive_serial_interface_[joint_id]->GetError();
			if(response != 0)
				ROS_ERROR("%s: Axis Error Status: %ld", joint_names_[joint_id].c_str(), response);

			//odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_IDLE, STATE_RQST_WAIT);

			ROS_INFO("Initialised O-Drive Serial Interface for %s", joint_names_[joint_id].c_str());
		}
		// Initialise Service Server to receive request to Initialise the Joints to Homing Position
		joint_homing_srv_ = nh_.advertiseService("odrive_control/joint_init_to_home", &ODriveHWInterface::joint_init_to_home, this);
		joint_idle_srv_ = nh_.advertiseService("odrive_control/joint_init_to_idle", &ODriveHWInterface::joint_init_to_idle,this); //added by ribin for idle funtion
		pause_robot_srv_ = nh_.advertiseService("odrive_control/pause_joints",&ODriveHWInterface::pause_joints,this); //added by ribin for pause function


             // set the gpio pins of the limit switch high 


                pi = pigpio_start(NULL,NULL);
	        if (pi < 0 ) {
                	ROS_ERROR("pigpio_start() function did not successful") ;
                //exit(1) ;
        	}
 		set_mode(pi,5,PI_INPUT);
                set_mode(pi,16,PI_INPUT);
                set_mode(pi,6,PI_INPUT);
                set_mode(pi,26,PI_INPUT);

		set_pull_up_down(pi,5,PI_PUD_UP);
	        set_pull_up_down(pi,16,PI_PUD_UP);
        	set_pull_up_down(pi,6,PI_PUD_UP);
        	set_pull_up_down(pi,26,PI_PUD_UP);

              /*  wiringPiSetupGpio();
                pinMode(5,INPUT);
                pullUpDnControl(5,PUD_UP);
                pinMode(6,INPUT);
                pullUpDnControl(6,PUD_UP);
                pinMode(16,INPUT);
                pullUpDnControl(16,PUD_UP);
                pinMode(26,INPUT);
                pullUpDnControl(26,PUD_UP);
              */

	}

	bool ODriveHWInterface::pause_joints(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp)
	{
		int joint_id = 0;
		ROS_INFO("%s, Joint PAUSE request Recieved", joint_names_[joint_id].c_str());
		return true;
	}


	bool ODriveHWInterface::joint_init_to_idle(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp)  //added by ribin for initialise joints to idle posiiton
	{
		Block_Read_Write_During_Initialization =1; //setting the flag high so the read and write functions wont be executed
		long response = 0;
		int joint_id = 0;
		joint_id = (int)req.joint_id;
		ROS_INFO("%s, Joint IDLE request Recieved", joint_names_[joint_id].c_str());
		ros::Duration(1).sleep();

		// Check the Axis Error
		/*  if(odrive_serial_interface_[joint_id]->GetError() != 0)
		    {
		    resp.reason = "Axis error \n";
		    ROS_ERROR("%s, Axis Error: %ld", joint_names_[joint_id].c_str(), response);
		    return false;
		    } */
		if(odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_IDLE, STATE_RQST_WAIT) != true)
		{
			resp.reason = "Unable to enter idle state \n";
			ROS_ERROR("%s, Unable to enter idle state", joint_names_[joint_id].c_str());
			return false;
		}
		Block_Read_Write_During_Initialization = 0; //setting the flag to zero so that  the read and write functions can work
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
		float target_step = 0.122;
		float target_step_l2 = 3.05; //edited by ribin for moving the l2  faster than other joints


		joint_id = (int)req.joint_id;
                   
                //initialising wiringpi for limit switch gpio



		//TODO//( l5 l3 should be initialised before l4 second)


		ROS_INFO("%s, Initialisation Request Recieved", joint_names_[joint_id].c_str());

		// Check the Axis Error
		if(odrive_serial_interface_[joint_id]->GetError() != 0)
		{
			resp.reason = "Axis error \n";
			ROS_ERROR("%s, Axis Error: %ld", joint_names_[joint_id].c_str(), response);
			return false;
		}

		// Set the Encoder is_ready to true
		//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_ENCODER_IS_READY_RW, true);

		//added by ribin for checking the limit swithc status before starting the encoder search for reversing the direction
		//joint_homing_switch_in_[joint_id].call(srv);   // commenting for limit switch remove
		
                //if (srv.response.success == true)
                //int limit_switch_gpio_value = digitalRead(limit_switch_id_[joint_id]);
                int limit_switch_gpio_value = gpio_read(pi,limit_switch_id_[joint_id]);
		if (limit_switch_gpio_value == 0)
		{
			ROS_INFO("Limits switch hit, reversing index search joint id:= %s", joint_names_[joint_id].c_str());
			double reverse_vel = -40;
			double reverse_accel = -20;
			double reverse_ramp_distance = -3.1415927410125732;
			odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_VEL, reverse_vel);  //these 3 values are in negative for reversing the index search
			odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_ACCEL, reverse_accel);
			odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_RAMP_DISTANCE, reverse_ramp_distance);
		}


		// Run the state to Index Search and wait for Idle
		if(odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_ENCODER_INDEX_SEARCH, STATE_RQST_NO_WAIT) != true)
		{
			resp.reason = "Unable to enter Encoder Search \n";
			ROS_ERROR("%s, Unable to enter Encoder Search", joint_names_[joint_id].c_str());
			return false;
		}
		ros::Duration(3).sleep();
		ROS_INFO("AXIS_STATE_ENCODER_INDEX_SEARCH finished");
		// Run the state to Closed Loop Control, and not return
		if(odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
		{
			resp.reason = "Unable to enter in Closed Loop Control\n";
			ROS_ERROR("%s, Unable to enter in Closed Loop Control", joint_names_[joint_id].c_str());
			return false;
		}
		ROS_INFO("AXIS_STATE_CLOSED_LOOP_CONTROL finished");

		// Check whether index was really found
		odrive_serial_interface_[joint_id]->ReadProperty(ODRV_INDEX_FOUND_R, &response);

		if(response == 0)
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
		//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW, (min_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id])/60);
		target = reference_target_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];

		//target_step *= direction_[joint_id];
		while(fabs(target - target_tmp) > 0.0)
		{
			long error = odrive_serial_interface_[joint_id]->GetError();
			if(error != 0)
			{
				ROS_ERROR("Error: %ld", error);
				return false;
			}
			if(target < 0 &&  joint_id !=3)
			{
				target_tmp -= target_step;
				std::cout<<target_step<<std::endl;
				ROS_INFO("Just for debug :%d",target_tmp);
			}
			//*********edited by ribin**********we have added condition for checking the initilaisation is for l2 or not, and if l2 we are changing the target_step to target_step_l2
			if(target >=0 && joint_id !=3)
			{
				target_tmp += target_step;
			}

			if(target <0 && joint_id ==3)
			{
				target_tmp -=target_step_l2;
				std::cout<<target_step<<std::endl;
				ROS_INFO("Just for debug :%d",target_tmp);
			}
			if(target >=0 && joint_id ==3)
			{
				target_tmp +=target_step_l2;
				std::cout<<target_step<<std::endl;
				ROS_INFO("Just for debug :%d",target_tmp);
			}


			//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, target_tmp);
			//modified with trajectory
			int motor_id = axis_id_[joint_id];
			odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE,motor_id,target_tmp);

			sleep(1);

			//joint_homing_switch_in_[joint_id].call(srv); // commenting for limit switch remove
                	limit_switch_gpio_value = gpio_read(pi,limit_switch_id_[joint_id]);
		        //limit_switch_gpio_value = digitalRead(limit_switch_id_[joint_id]);

                        //if(srv.response.success == true)
			if (limit_switch_gpio_value == 0)
                           	break;
			else
				ROS_INFO("%s: Limit Switch not hit, target: %ld", joint_names_[joint_id].c_str(), target_tmp);
		}
		// Stop the motor
		//odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &zero_offset_[joint_id]);
		odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &zero_offset_[joint_id]);
		//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
		//modified by trajectory 
		int motor_id = axis_id_[joint_id];
		odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE,motor_id,zero_offset_[joint_id]);



		ROS_INFO("Zero Offset at limit_switch_hit_pos: %ld", zero_offset_[joint_id]);
		// joint3 id is 0
		if(joint_id ==0){
			zero_offset_[joint_id] += homing_pos_init_seq_[0] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];
			ROS_INFO("zero_offset_ at homing_pos_init_seq_ %ld",  zero_offset_[joint_id]);
		}

		// Adding the homing position to zero_offset_
		else{
			zero_offset_[joint_id] += homing_position_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id];
			ROS_INFO("Zero offser at homing_position: %ld", zero_offset_[joint_id]);
		}
		// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, zero_offset_[joint_id]);
		//   int motor_id = axis_id_[joint_id];
		odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE,motor_id,zero_offset_[joint_id]);
		// Joint4 id = 1
		if(joint_id ==1)
		{
			int JOINT3_ID = 0 ;
			int JOINT3_MOTOR_ID = 0 ;
			// Move Joint3 to the Joint3 homing_postion instead of the JOINT3 homing_pos_init_seq
			sleep(1);
			zero_offset_[JOINT3_ID ] += homing_position_[JOINT3_ID ] * transmission_factor_[JOINT3_ID ] * encoder_resolution_[JOINT3_ID ] * direction_[JOINT3_ID ];
			// odrive_serial_interface_[0]->WriteProperty(ODRV_POSITION_RW, zero_offset_[0]);
			//modified for trajectory
			//int motor_id = axis_id_[joint_id];
			odrive_serial_interface_[JOINT3_ID]->Write_command_float(ODRV_MOVE_TO_POSE,JOINT3_MOTOR_ID,zero_offset_[JOINT3_ID]);


			ROS_INFO("Zero offser at homing_position: %ld", zero_offset_[0]);

		}
		//    ROS_INFO("Zero Offset: %ld", zero_offset_[joint_id]);

		// Set the max. velocity to corresponding max. value
		//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_VELOCITY_RW, (max_velocity_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id])/60);

		++No_Of_Joint_initialised_to_home_;
		Block_Read_Write_During_Initialization = 0; //setting the flag to 0 so that the read and write can function without any problem
		return true;
	}

	void ODriveHWInterface::read(ros::Duration &elapsed_time)
	{
		dynamixel_msgs::JointState joint_state;

		// Run for all the Joints
		for(std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
		{
			//  if(No_Of_Joint_initialised_to_home_ >= num_joints_)
			if(Block_Read_Write_During_Initialization ==0){     // the calling flag is used for blocking the execution of read and write during the initialisation service



				//><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<added by ribin on 10/03/2020 for stol error>>>>>>>>>>>>>>>>>>
				if (height_scan_enable ==true) {
					if(No_Of_Joint_initialised_to_home_ > 3)
					{
						// Read current Position
						double joint_current_pos = 0.0;
						//odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
						odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &joint_current_pos);
						//  ROS_WARN("is this the one being called from odrive_hw_interface");
						joint_current_pos -= zero_offset_[joint_id];
						joint_current_pos *= direction_[joint_id];
						joint_position_[joint_id] = (joint_current_pos / transmission_factor_[joint_id]) / encoder_resolution_[joint_id];
					}

				}

				if (height_scan_enable ==false) {
					if(No_Of_Joint_initialised_to_home_ >= 3)
					{
						// Read current Position
						double joint_current_pos = 0.0;
						//odrive_serial_interface_[joint_id]->ReadProperty(ODRV_CURRENT_POSITION_R, &joint_current_pos);
						odrive_serial_interface_[joint_id]->ReadProperty(ODRV_POS_ESTIMATE, &joint_current_pos);
						//  ROS_WARN("is this the one being called from odrive_hw_interface");
						joint_current_pos -= zero_offset_[joint_id];
						joint_current_pos *= direction_[joint_id];
						joint_position_[joint_id] = (joint_current_pos / transmission_factor_[joint_id]) / encoder_resolution_[joint_id];

					}
				}

			}
			joint_state_pub_[joint_id].publish(joint_state);
		}
	}

	void ODriveHWInterface::write(ros::Duration &elapsed_time)
	{
		// Safety
		enforceLimits(elapsed_time);
		for(std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
		{
			if(Block_Read_Write_During_Initialization ==0)
			{     // the calling flag is used for blocking the execution of read and write during the initialisation service

				// So that we should not send commands to O-Drive repeatedly
				//<<<<<<<<<<<<<<<<<<<<<<<< edited by ribin to make sure we dont have stol error on 10/03/2020>>>>>>>>>>>>>>

				/*
				   if(joint_position_command_[joint_id] != last_joint_position_command_[joint_id]
				   && (No_Of_Joint_initialised_to_home_ >=3)) //edited ribin for l2 disable
				   */

			//sometime the height scan is not enabled  so we have
                                long error = odrive_serial_interface_[joint_id]->GetError();
                                if(error != 0)
                                {
			
                                ROS_ERROR("OdriveHWInterface::write joint no %d ERROR : %ld", joint_id, error);
				ROS_ERROR("OdriveHWInterface::write Shutting Down ROS , calling ros::shutdown()") ;
                                system("rosnode kill -a");
				ros::shutdown() ;
                                }

				if (height_scan_enable ==true) {

					if(joint_position_command_[joint_id] != last_joint_position_command_[joint_id]
							&& (No_Of_Joint_initialised_to_home_ >3))

						//  && (No_Of_Joint_initialised_to_home_ >= num_joints_))
					{
						//edited by ribin for making the l2 to axis state closed loop
						if(joint_id==3)
						{
							if(odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
							{
								//resp.reason = "Unable to enter in Closed Loop Control\n";
								ROS_ERROR("%s, Unable to enter in Closed Loop Control", joint_names_[joint_id].c_str());
								//return false;
							}
						}

						ROS_INFO("Writing to odrive for joint =: %s",joint_names_[joint_id].c_str());

						ROS_WARN("%s: Command: %lf, Last Command: %lf", joint_names_[joint_id].c_str(), joint_position_command_[joint_id], last_joint_position_command_[joint_id]);
						// Check the Joint Limits read from the URDF
						if((joint_position_command_[joint_id] >= joint_position_lower_limits_[joint_id])
								&& (joint_position_command_[joint_id] <= joint_position_upper_limits_[joint_id]))
						{
							float output_value = 0.0;
							//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW,
							//                                                  (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id])
							//                                                   + zero_offset_[joint_id]);
							ROS_INFO("joint_position_command_ := %d",joint_position_command_[joint_id]);
							output_value = (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]) + zero_offset_[joint_id];
							ROS_INFO("%s: Output Value: %lf", joint_names_[joint_id].c_str(), output_value);

							//important we are gonna command out this  function, which is responsible for moving the arm
							//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, output_value);
							//now we are gonna replace it with Trajectory_write
							int motor_id = axis_id_[joint_id]; 
                                                        ROS_INFO("MOTOR ID %i and joint id %i ",motor_id,joint_id);
							odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE,motor_id,output_value);
							ROS_INFO("Command Sent to Port: %d, Motor: %d, Error: %ld", odrive_id_[joint_id], axis_id_[joint_id], odrive_serial_interface_[joint_id]->GetError());
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
				}


				if (height_scan_enable ==false) {

					if(joint_position_command_[joint_id] != last_joint_position_command_[joint_id]
							&& (No_Of_Joint_initialised_to_home_ >=3))
					{
						if(joint_id==3)
						{
							if(odrive_serial_interface_[joint_id]->Run_State(AXIS_STATE_CLOSED_LOOP_CONTROL, STATE_RQST_NO_WAIT) != true)
							{
								//resp.reason = "Unable to enter in Closed Loop Control\n";
								ROS_ERROR("%s, Unable to enter in Closed Loop Control", joint_names_[joint_id].c_str());
								//return 0;
							}
                                		long error = odrive_serial_interface_[joint_id]->GetError();
                                		}

						//Todo RIBIN for joint_id==3 we have to make

						ROS_INFO("Writing to odrive for joint =: %s",joint_names_[joint_id].c_str());

						ROS_WARN("%s: Command: %lf, Last Command: %lf", joint_names_[joint_id].c_str(), joint_position_command_[joint_id], last_joint_position_command_[joint_id]);
						// Check the Joint Limits read from the URDF
						if((joint_position_command_[joint_id] >= joint_position_lower_limits_[joint_id])
								&& (joint_position_command_[joint_id] <= joint_position_upper_limits_[joint_id]))
						{
							float output_value = 0;
							//odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW,
							//                                                  (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id])
							//                                                   + zero_offset_[joint_id]);
							ROS_INFO("joint_position_command_ := %d",joint_position_command_[joint_id]);
							output_value = (joint_position_command_[joint_id] * transmission_factor_[joint_id] * encoder_resolution_[joint_id] * direction_[joint_id]) + zero_offset_[joint_id];
							ROS_INFO("%s: Output Value: %lf", joint_names_[joint_id].c_str(), output_value);

							//important we are gonna command out this  function, which is responsible for moving the arm
							//******************** TODO pose set point //
							// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_POSITION_RW, output_value);
							//now we are gonna replace it with Trajectory_write
							//Edited on 28/08/2020 for trajecotry movement/

							int motor_id = axis_id_[joint_id]; 
							ROS_INFO("MOTOR ID %i and joint id %i ",motor_id,joint_id);
							odrive_serial_interface_[joint_id]->Write_command_float(ODRV_MOVE_TO_POSE,motor_id,output_value);

							ROS_INFO("Command Sent to Port: %d, Motor: %d, Error: %ld", odrive_id_[joint_id], axis_id_[joint_id], odrive_serial_interface_[joint_id]->GetError());
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



				}
				last_joint_position_command_[joint_id] = joint_position_command_[joint_id];
			}
		}
	}

	void ODriveHWInterface::enforceLimits(ros::Duration &period)
	{
		// Enforces position and velocity
		//pos_jnt_sat_interface_.enforceLimits(period);
	}

}   // namespace odrive_control
