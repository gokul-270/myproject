#History : 5Jan2021
#  Added ability to read aruco markers (No 23) so that it is good for testing in the lab
#  without the need for testing cotton in the lab.

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <dynamixel_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <yanthra_move/yanthra_move.h>
#include <yanthra_move/joint_move.h>
#include <yanthra_move/yanthra_io.h>
#include "cotton_detection/capture_cotton_srv.h"
#include <sys/time.h>
#include <time.h>

double start_time, end_time, last_time; 

bool height_scan_enable;
double height_scan_min;
double height_scan_max;
double height_scan_step;
bool Global_vaccum_motor;
double joint5_vel_limit;
float l2_homing_sleep_time;
float l2_step_sleep_time;
bool jerk_enabled_phi;
bool jerk_enabled_theta;
float theta_jerk_value;
float phi_jerk_value;
bool YanthraLabCalibrationTesting = true;
int picked = 0;
double joint2_old_pose = 0.001;
ros::ServiceClient cotton_client;
/*
   MAIN PROGRAM
   */



static void print_timestamp(const char *msg)
{
	time_t ct = { 0 };
	struct tm lut_tm = { 0 };
	char lut[30] = { 0 };

	assert(msg && (strlen(msg) > 0));
	ct = time(NULL);
	ROS_INFO("%s at %s\n", msg, asctime_r(localtime_r(&ct, &lut_tm), lut));
}




double currentTimeMillis()
{
	struct timeval timeVar ;
	gettimeofday(&timeVar, NULL);
	double s1 = (long) (timeVar.tv_sec) * 1000;
	double s2 = (timeVar.tv_usec/1000);
	return(s1+s2);
}





int main (int argc, char** argv)
{
	ros::init(argc, argv, "yanthra_move");
	ros::NodeHandle n;

	/* Wait for node to start */
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate loop_rate(100);
        XmlRpc::XmlRpcValue joint_pose_values;
	//moveit::planning_interface::MoveGroup group("Arm");

	/* Reading Configuration */
	n.param<bool>("continous_operation", continous_operation, true);
	n.param<float>("delays/picking", picking_delay, 0.5);
	n.param<float>("delays/pre_start_len", pre_start_len, 0.010);
	n.param<bool>("jerk_enabled_theta",jerk_enabled_theta,true);
	n.param<bool>("jerk_enabled_phi",jerk_enabled_phi,true);

	/* Joint3 configuration */
	n.param<double>("joint3_init/park_position", joint3_parking_pose, 0.0001);
	n.param<double>("joint3_init/homing_position", joint3_homing_position,0.001 );
	n.param<bool>("/joint3_init/multiple_zero_poses", joint3_multiple_zero_pose, true);

	ROS_INFO( "Joint3 Homing Position %d ", (int)joint3_homing_position) ;

	/* Joint 4 configuration */
	n.param<bool>("/joint4_init/multiple_zero_poses", joint4_multiple_zero_pose, true);
	ROS_INFO("multiple_zero_poses value read form config file := %d",joint4_multiple_zero_pose);

	n.param<double>("/joint4_init/park_position", joint4_parking_pose, 0.001);
	n.param<double>("joint4_init/homing_position", joint4_homing_position,0.001 );
	n.param<float>("joint4_init/theta_jerk_value",theta_jerk_value,0.0);

	n.param<float>("joint5_init/phi_jerk_value",phi_jerk_value,0.0);
	n.param<float>("l2_step_sleep_time",l2_step_sleep_time,5.0);
	n.param<float>("l2_homing_sleep_time",l2_homing_sleep_time,5.0);
	n.getParam("joint_poses",joint_pose_values);
	ROS_ERROR("%f",joint_pose_values[0][1]);
        int size_of_pose =joint_pose_values.size();
        ROS_ERROR("%d",size_of_pose);
        for(int i=0; i<size_of_pose; i++ )
        {

                //pose values read from yaml file

                ROS_INFO("link2 poses %f:=",joint_pose_values[i][0]);
                double joint2_pose =  joint_pose_values[i][0];
                double joint3_pose  = joint_pose_values[i][1];
                double joint4_pose  = joint_pose_values[i][2];
                ROS_INFO("link2 poses %f:=",joint2_pose);
                ROS_INFO("link2 poses %f:=",joint3_pose);
                ROS_INFO("link2 poses %f:=",joint4_pose);

        }

	d_out vaccum_motor(n,"vaccum_motor");


	if(joint3_multiple_zero_pose==true)
	{
		n.getParam("/joint3_init/zero_poses", joint3_zero_poses);
		ROS_INFO("multiple_zero_poses value read form config file := %f, %f",joint3_zero_poses[0],joint3_zero_poses[1]);

	}

	else {
		joint3_zero_poses.push_back(0.0);
	}


	if(joint4_multiple_zero_pose == true)
	{
		n.getParam("/joint4_init/zero_poses", joint4_zero_poses);
		ROS_INFO("multiple_zero_poses value read form config file := %f, %f",joint4_zero_poses[0],joint4_zero_poses[1]);
	}

	else{
		joint4_zero_poses.push_back(0.0);
	}

	ROS_INFO("Joint4: Total Zero poses: %d", (int)joint4_zero_poses.size());
	ROS_INFO("joint3: Total Zero poses: %d",(int)joint3_zero_poses.size());
	/* Joint 5 configuration */
	n.param<double>("joint5_init/park_position", joint5_parking_pose, 0.0001);
	n.param<double>("joint5_init/homing_position", joint5_homing_position,0.001 );
	n.param<double>("joint5_init/end_effector_len", end_effector_len, 0.095);
	n.param<double>("joint5_init/min_length", link5_min_length, 0.313);
	n.param<double>("joint5_init/max_length", link5_max_length, 0.602);
	n.param<double>("joint5_init/joint5_vel_limit", joint5_vel_limit, 0.55);

	n.param<bool>("joint2_init/height_scan_enable",height_scan_enable,true);
	n.param<bool>("global_vaccum_motor",Global_vaccum_motor,true);
	ROS_INFO("height_scan_enable value read form config file := %d",height_scan_enable);
	/*#if HEIGHT_SCAN_EN == true
	  double height_scan_min;
	  double height_scan_max;
	  double height_scan_step;
#endif*/

	//#if HEIGHT_SCAN_EN == true
	n.param<double>("joint2_init/min", height_scan_min, 0.01);
	n.param<double>("joint2_init/max", height_scan_max, 1.000);
	n.param<double>("joint2_init/step", height_scan_step, 0.200);
	//height_scan_value = height_scan_step;

	servo_out servo_joint_2(n, "joint2_move", height_scan_min, height_scan_max);  //commmented out by ribin this function is used for initialisation of servo,
	//the initialisation of odrive is done by adding the joint Parameters in the config file
	//so we dont need to initialise from here
	//#endif

#if SECTOR_SCAN_EN == true
	servo_out servo_joint_1(n, "joint1_move", JOINT1_CONV_FACTOR);
	servo_joint_1.init();
#endif

	joint_move joint_move_3(n, "joint3_position_controller/");
	joint_move joint_move_4(n, "joint4_position_controller/");
	joint_move joint_move_5(n, "joint5_position_controller/");

	/*####################here we are gonna add the call for l2 to initialised##################*/
	joint_move joint_move_2(n, "joint2_position_controller/");   //edited by ribin this is the joint for l2
	/*##########################initialisation of l2 compleated################################*/

	joint_move::joint_pub_trajectory = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1000);
	joint_move::joint_homing_service = n.serviceClient<odrive_control::joint_homing>("/odrive_control/joint_init_to_home");
	//edited by ribin for making the l2 idle
	joint_move::joint_idle_service = n.serviceClient<odrive_control::joint_homing>("/odrive_control/joint_init_to_idle");
	/* Relay switches output */

	//edited by ribin for calling cotton_detectin_service on 20_04_2020	
	joint_move::cotton_detection_ml_service = n.serviceClient<cotton_detection::capture_cotton_srv>("/capture_cotton");	
	ROS_INFO("subscribed to capture");



#if END_EFFECTOR_EN == true
#if AGGREGATE_PICK_EN == false
	d_out front_valve(n, "front_valve");
	d_out back_valve(n, "back_valve");
	d_out end_effector(n, "end_effector");
#elif AGGREGATE_PICK_EN == true
	a_out pick_cotton(n, "pick_cotton");
	a_out drop_cotton(n, "drop_cotton");
	a_out lid_open(n, "lid_open");
#endif

#endif
	/* LED and Polariser working if Camera is enabled */
#if CAMERA_EN == true
	d_out led_control(n, "led_control");

#endif
	/* Limit Switch I/P for Joint 5 Initialisation */
#if JOINT5_INIT_EN == true
	d_in joint5_switch_in(n, "joint5_switch_in");
	d_out joint5_init_start(n, "joint5_init_start");
#endif
	/* START_SWITCH_IN and OUTPUT_INDICAATOR_LED for User to Start Picking operation */
#if START_SWITCH_EN == true
	d_in start_switch_in(n, "start_switch_in");
	d_out start_switch_out(n, "start_switch_out");
#endif
	d_out problem_indicator_out(n, "problem_indicator_out");
	/* SHUTDOWN_SWITCH_IN, if start_stop switch is pressed for more than 5sec, initiate shutdown */
#if SHUTDOWN_SWITCH_EN == true
	d_in shutdown_switch_in(n, "shutdown_switch_in");
#endif




	/* INITIALISATION */


	ROS_WARN("Starting Initialisation\n\n");

	/* Cotton Coordinates */
	std::vector<geometry_msgs::Point> positions;
	std::vector<geometry_msgs::PointStamped> positions_link3;

	/* Find the transformed coordinates from camera_reference_frame to link3 */
	tf::StampedTransform tf_camera_base;
	tf::TransformListener listener_camera_base;

	try
	{
		listener_camera_base.waitForTransform("/link3", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(30.0));
		listener_camera_base.lookupTransform("/link3", "/camera_depth_optical_frame", ros::Time(0), tf_camera_base);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	ROS_INFO("Difference between Camera and Link3, x: %lf, y: %lf, z: %lf",
			tf_camera_base.getOrigin().x(),
			tf_camera_base.getOrigin().y(),
			tf_camera_base.getOrigin().z());

	/* Wait for callback to be called */
	ros::Duration(5.0).sleep();



#if BREAD_BOARD == true
	return 0;
#endif
	std::string throwaway;
#if START_SWITCH_EN == true
	start_switch_out.command(true);
	ROS_WARN("Initialisation complete, \n Press START_SWITCH to start the Robot...\n\n");
	bool STARTSWITCH,SHUTDOWNSWITCH;
	while(  (!(SHUTDOWNSWITCH=shutdown_switch_in.state())) && (!(STARTSWITCH = start_switch_in.state())) )
	{
		ros::spinOnce();
	}
	start_switch_out.command(false);
	ROS_INFO( "StartSwitch %d SHUTDOWNSWITCH %d \n", STARTSWITCH, SHUTDOWNSWITCH) ;
	if (SHUTDOWNSWITCH)
	{
		ROS_ERROR("SHUTTING  DOWN ROS AND SYSTEM");
		system("sudo poweroff");
		ros::shutdown() ; // Shutting down ROS
		//system("sudo poweroff");
	}

	ROS_INFO( "Got STARTSWITCH COMMAND Continuing the program ... \n") ;
#else
	ROS_WARN("Initialisation complete, \n Press ENTER to start the Robot...(type 'return' to terminate the process)\n\n");
	std::getline(std::cin, throwaway);
	if(throwaway.compare("return") == 0)
		return 0;
#endif

	/*From ROS to ODRIVE HOMING REQUEST*/
	//TODO The Odrive Homing Position is Independent of main program

	// TODO: Put a check  whether the services are available
	odrive_control::joint_homing srv;
	odrive_control::joint_homing srv_idle;

	//edited by ribin for checking the cotton_detection_server
	cotton_detection::capture_cotton_srv cotton_detection_srv;


	//added by ribin for changing the sequance to height scan first
	//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
	if (height_scan_enable ==true) {
		srv.request.joint_id = 3;
		srv_idle.request.joint_id = 3;
		if(joint_move::joint_homing_service.call(srv)!=true)
		{
			ROS_ERROR("Joint2 Homing, Reason: %s", srv.response.reason.c_str());
			problem_indicator_out.command(true);
			return 0;
		}
                  
                ros::Duration(10).sleep();
		if(joint_move::joint_idle_service.call(srv_idle)!=true)
		{
			ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
			problem_indicator_out.command(true);
			return 0;
		}


	}
	//>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
	// Joint5 Homing (r, i.e. Prismatic Joint)
	srv.request.joint_id = 2;
	if(joint_move::joint_homing_service.call(srv) != true)
	{
		ROS_ERROR("Joint5 Homing, Reason: %s", srv.response.reason.c_str());
		problem_indicator_out.command(true);
		return 0;
	}

	// Joint3 Homing (phi, i.e. Revolute Joint sweeping along vertical)
	srv.request.joint_id = 0;
	if(joint_move::joint_homing_service.call(srv) != true)
	{
		ROS_ERROR("Joint3 Homing, Reason: %s", srv.response.reason.c_str());
		problem_indicator_out.command(true);
		return 0;
	}

	// Joint4 Homing (theta, i.e. Revolute Joint sweeping along horizontal)
	srv.request.joint_id = 1;
	if(joint_move::joint_homing_service.call(srv) != true)
	{
		ROS_ERROR("Joint4 Homing, Reason: %s", srv.response.reason.c_str());
		problem_indicator_out.command(true);
		return 0;
	}
	//chaned by ribin on 10/03/2020
	//Changing the sequance of initialisation to l2 first so moved this poriton of the code first
	/*
	   }
	   */

ROS_INFO("All joints are initialised");

/* Move Joint3 and Joint4 to their default pose */
//joint_move_3.move_joint(0.0, WAIT);
//joint_move_4.move_joint(0.0, WAIT);

/* While ROS OK */
while(ros::ok())
{

#if START_SWITCH_EN == true
	ROS_WARN("Ready to go to another cycle, \n Press START_SWITCH start the Robot...\n\n");
	start_switch_out.command(true); // Switch On the Red bulb indicator

	while(  (!(SHUTDOWNSWITCH=shutdown_switch_in.state())) && (!(STARTSWITCH = start_switch_in.state())) )
	{
		ros::spinOnce();
	}
	start_switch_out.command(false);// Switch OFF the red bulb user indicator
	ROS_INFO( "StartSwitch %d SHUTDOWNSWITCH %d \n", STARTSWITCH, SHUTDOWNSWITCH) ;
	if (SHUTDOWNSWITCH)
	{
		ROS_ERROR("SHUTTING DOWN ROS and Powering down ");
		system( "sudo poweroff") ;
		ros::shutdown() ; // Shutting down ROS
		//system( "sudo poweroff") ;
	}

	ROS_INFO( "Got STARTSWITCH COMMAND Continuing the program ... \n") ;
#else
	ROS_WARN("Ready to go to another cycle, \n Press ENTER to start the Robot...(type 'return' to terminate the process)\n\n");
	std::getline(std::cin, throwaway);
	if(throwaway.compare("return") == 0)
		return 0;
#endif


	/* **** Height SCAN **** */
	/*####################here we are gonna add the call for l2 to initialised##################*/

	/*##########################initialisation of l2 compleated################################*/


	print_timestamp("height sacn started");

	int number_of_steps =0;

	if (height_scan_enable ==true) {
		number_of_steps = height_scan_max /height_scan_step;
	} // 10/2 =5 //0.4 /0.15
	else{
		number_of_steps =0;
	}
	double height_scan_value = height_scan_min;

	start_time = currentTimeMillis(); 
	double time_taken_between_height_scan = start_time - last_time; //computed in milli seconds	

	ROS_INFO("TIMETO taken between height_scan = %f ",time_taken_between_height_scan);	
	
        for(int i=0; i<size_of_pose; i++ )
        {

                //pose values read from yaml file

                double joint2_pose =  joint_pose_values[i][0];
                double joint3_pose  = joint_pose_values[i][1];
                double joint4_pose  = joint_pose_values[i][2];
                ROS_INFO("link2 poses %f:=",joint2_pose);
                ROS_INFO("link3 poses %f:=",joint3_pose);
                ROS_INFO("link2 poses %f:=",joint4_pose);
                ROS_INFO("link2 old poses %f:=",joint2_old_pose);

               if (height_scan_enable ==true) {
			if (joint2_pose < height_scan_max)
			{
			   if(joint2_old_pose != joint2_pose){

				ROS_ERROR("inside");

		                joint_move_4.move_joint(joint4_homing_position, WAIT); // TODO move it to joint4_homing_position
				ros::Duration(2).sleep();
				joint_move_2.move_joint(joint2_pose,WAIT);
				ros::Duration(l2_step_sleep_time).sleep();
				ROS_INFO("height_scan_step value %f:=", joint2_pose);
				srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
				if(joint_move::joint_idle_service.call(srv_idle)!=true)
				{
					ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
					problem_indicator_out.command(true);
					return 0;
				}
				//		ROS_INFO("height_scan_value %f:=", height_scan_value);
				
				

			}
			joint2_old_pose = joint2_pose;
		      }

		}
		



			joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
			ros::Duration(2).sleep();
			joint_move_3.move_joint(joint3_pose,WAIT);
			ros::Duration(2).sleep();
			joint_move_4.move_joint(joint4_pose, WAIT);

			/* Initiate a camera start request */
#if CAMERA_EN == true
		if (!YanthraLabCalibrationTesting) {
			ros::Duration(3).sleep();
			led_control.command(START);
			//vaccum_motor.command(START); //added by ribin for controlling the blower motor
			ROS_INFO("Executing new program: " COTTON_DETECT_PROGRAM " " YANTHRA_DATA_INPUT_DIR "/ " YANTHRA_DATA_OUTPUT_DIR "/");
			cotton_detection_srv.request.capture = true;
		        ros::Duration(2).sleep();
                                        

			// edited by ribin for converting the system call to service call
			/*
			   if(joint_move::cotton_detection_ml_service.call(cotton_detection_srv) != true)

			   {
			   ROS_INFO("cotton_detection_service called");

			   }
			   */
			if(joint_move::cotton_detection_ml_service.call(cotton_detection_srv) != true)
			{
				ROS_ERROR("Cotton Detection Failed: ");
				problem_indicator_out.command(true);
			}


			/*	if(state != 0)
				{
				ROS_ERROR("Cotton Detection Failed: %d", state);
				problem_indicator_out.command(true);
				}
				*/
			led_control.command(STOP);
			record_debug_data();		// Store the captured images in debug folder
#endif
			/* Get the coordinates of cotton */
			ROS_INFO("Read the coordinates");
			get_cotton_coordinates(&positions);

			/* ikkConvert cotton coordinates from camera_frame to link3 frame */
#if CAMERA_EN == true
			getCottonCoordinates_cameraToLink3(&listener_camera_base, &positions, &positions_link3);
#else
			getCottonCoordinates_ToLink3(&positions, &positions_link3);
#endif
		}
		else { // of YanthraLabCalibrationTesting	
		      //sprintf(FilePathName,"%s/result_image", PRAGATI_OUTPUT_DIR);
			sprintf(cotton_coordinate_filename ,"%s/cotton_details.txt", PRAGATI_OUTPUT_DIR);

			// Call ARUCO_FINDER_PROGRAM
			// Aruco_Finder put the location of the files in the file centroid.tx
			// Read data from centroid.txt and then move the arm

			system(ARUCO_FINDER_PROGRAM);

			/* Read  from the generated centroid.txt file from aruco_finder */
			std::ifstream mark_centroid("centroid.txt");
			if (!mark_centroid.is_open()) {
				ROS_INFO("Error: not able to open the file : centroid.txt ");
				ROS_INFO("Exiting yanthra_move ");
				ROS_ERROR("Error: not able to open the file : centroid.txt ");
				ROS_ERROR("Exiting yanthra_move ");
				exit(1) ;
			}

			std::ofstream cotton_fs(cotton_coordinate_filename.c_str());

			if (!cotton_fs.is_open()) {
				ROS_INFO("Error: not able to open the file : %s ", cotton_coordinate_filename.c_str());
				ROS_INFO("Exiting yanthra_move ");
				ROS_ERROR("Error: not able to open the file : %s ", cotton_coordinate_filename.c_str());
				ROS_ERROR("Exiting yanthra_move ");
				exit(1) ;
			}


			float XValue, YValue, ZValue;
			int PixelColumn = 0 ;
			int PixelRow = 0 ;
			while(mark_centroid >> x >> y >> z)
			{
				// Print to the cotton_details.txt file the four corners of the marker
				// These values are w.r.t to the Camera origin.
				// When using in the arm this has to be converted to ARM's origin.

				cotton_fs << PixelColumn <<" " << PixelRow << " " <<" "<< (float) XValue << " " <<(float)YValue << " " << (float) ZValue << std::endl ;
				cotton_fs.flush();
				ROS_INFO( "( XValue : %.2f, YValue :  %.2f, ZValue : %.2f)", XValue, YValue, ZValue) ;
			}
			// Close the output steam
			cotton_fs.close();
			mark_centroid.close();

			// Reading the CottonCoordinate from a file
			get_cotton_coordinates(&positions);
			getCottonCoordinates_cameraToLink3(&listener_camera_base, &positions, &positions_link3);
		} //  of -- if (YanthraLabCalibration) 
	
                                        if(Global_vaccum_motor == true)
                                        {
                                                vaccum_motor.command(START); //added by ribin for controlling the blower motor
                                                //TODO check whether the motor is still on or not



                                        }
			// here we are gonna pick all cotton in current pose, the positions_link3 variable is loaded with positions of each cotton which is validated for reachability 
			int trgt = 0;
		//	int picked =0;
			double  start_time_to_pick_n_number_of_cotton = currentTimeMillis();
			for(std::vector<geometry_msgs::PointStamped>::iterator it = positions_link3.begin(); it != positions_link3.end(); it++)
			{

				double	r = 0,
					theta = 0,
					phi = 0;

				/* If targets to be reached in steps, rather continous operation */
				if(continous_operation != true)
				{
					ROS_WARN("Press ENTER to move to next target...(type 'done' to terminate the process)\n\n");
					std::getline(std::cin, throwaway);
					if(throwaway.compare("done") == 0)
						break;
				}
				ROS_WARN("Moving to target %d\n", ++trgt);
				/* Convert XYZ coordinate to polar coordinates */
				xyz_to_polar(it->point.x, it->point.y, it->point.z, &r, &theta, &phi);
				ROS_WARN( " YANTHRA_MOVE found R : %f, theta : %f , phi :%f ",r, theta, phi) ;
				if(check_reachability(r, theta, phi) == true)
				{
					double picking_time_started = currentTimeMillis();
					joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
					float joint5_delay = (r/joint5_vel_limit);
					ros::Duration(joint5_delay).sleep();
					ROS_INFO("joint5 return delay: %f ", joint5_delay);
					ROS_ERROR("picked number: %d ", picked);


                                           
					joint_move_4.move_joint(joint4_pose + theta, WAIT);
					joint_move_3.move_joint(joint3_pose + phi,WAIT);
					float joint_velocity = (10000/8192)*2*3.14;
					float joint4_delay = joint4_pose/joint_velocity;
					float joint3_delay = joint3_pose/joint_velocity;
					if(joint4_delay < 0)
					{
						joint4_delay = joint4_delay * -1 ;

					}
					if(joint3_delay < 0) 
					{
						joint3_delay = joint3_delay * -1;
					}

					ROS_INFO("joint4_delay %f ", joint4_delay);            // joint4 delay
					ROS_INFO("joint3_delay %f ", joint3_delay);            // joint3 delay

					if (joint4_delay > joint3_delay)
					{
						ros::Duration(joint4_delay+0.2).sleep();

					}
					else
					{
						ros::Duration(joint3_delay+0.2).sleep();
					}

					r -= LINK5_MIN_LENGTH;
#if END_EFFECTOR_EN == true
#if AGGREGATE_PICK_EN == true
					float pre_start_delay = ((r - pre_start_len) / joint5_vel_limit);
					ROS_WARN("End-Effector will start in %f sec", pre_start_delay);
					ROS_INFO("End-Effector will start in %f sec", pre_start_delay);
					if (pre_start_delay <0)  //added by ribin for calling the pick cotton if the pre_start_delay is negative
					{
						pre_start_delay = 0;
					}

					pick_cotton.command(pre_start_delay);
                                      
#endif
#endif
					double link_5_start_time = currentTimeMillis();
					joint_move_5.move_joint(r, WAIT);
					float joint5_forward_delay = joint5_delay + picking_delay;

					ROS_INFO("joint5_forward_delay %f ", joint5_forward_delay);     // joint5 forward delay
					ros::Duration(joint5_forward_delay).sleep();
					double link_5_end_time = currentTimeMillis();
					double link_5_moving_time = link_5_end_time - link_5_start_time;
					ROS_INFO("TIMETO link_5_moving_time = %f ",link_5_moving_time);

					double end_picking_time =currentTimeMillis();
					picked++;
					double time_taken_for_each_pick = end_picking_time - picking_time_started;
					ROS_INFO("TIMETO pick one cotton by arm and EEffector = %f ",time_taken_for_each_pick);
					joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
					ros::Duration(joint5_delay).sleep();
				/*	if (picked == 5){
					
						  if(Global_vaccum_motor == true)
                                        	{
                                                vaccum_motor.command(STOP); //added by ribin for controlling the blower motor
                                                //TODO check whether the motor is still on or not
                                        	}
					  ROS_ERROR("inisde picking");
					  joint_move_3.move_joint(joint3_parking_pose,WAIT);
					 ros::Duration(1).sleep();
					  joint_move_4.move_joint(joint4_parking_pose,WAIT);
					  ros::Duration(1).sleep();

					  lid_open.command(2);
					  ros::Duration(3).sleep();
					  picked = 0;
					  joint_move_4.move_joint(joint4_zero_poses[joint4_cnt] , WAIT);
					  joint_move_3.move_joint(joint3_zero_poses[joint3_cnt] ,WAIT);
					  ros::Duration(1.5).sleep();
					
						  if(Global_vaccum_motor == true)
                                        	{
                                                vaccum_motor.command(START); //added by ribin for controlling the blower motor
                                                //TODO check whether the motor is still on or not
                                        	}
					}*/

                                       drop_cotton.command(joint5_delay);
				}
				else
				{
					ROS_ERROR("Cotton target %d is out of bound", trgt);
				}
				ros::spinOnce();
				if((joint_move_3.error_code != NO_ERROR) ||
						(joint_move_4.error_code != NO_ERROR) ||
						(joint_move_5.error_code != NO_ERROR))
				{
					ROS_ERROR("PROBLEM IN SOME JOINT");
					problem_indicator_out.command(true);
					break;
				} // if ((joint_move_3.error ...
			}
			end_time = currentTimeMillis();
			double time_taken_for_picking_n_number_of_cotton = end_time - start_time_to_pick_n_number_of_cotton;
			ROS_INFO("TIMETO pick %d number of cotton = %f",picked,time_taken_for_picking_n_number_of_cotton);
			//ROS_INFO("TIMETO pick %d number of cotton %f",picked,(int)time_taken_for_picking_n_number_of_cotton);
			ROS_INFO("NUMBEROF cotton picked = %d",picked);

			if(Global_vaccum_motor == true){
				vaccum_motor.command(STOP);	//here we are gonna publish a msg that gonna swithoff the blower
			}
			
				if (picked > 10){
					
					  ROS_ERROR("inisde picking");
					  joint_move_3.move_joint(joint3_parking_pose,WAIT);
					 ros::Duration(1).sleep();
					  joint_move_4.move_joint(joint4_parking_pose,WAIT);
					  ros::Duration(1).sleep();

					  lid_open.command(0);
					  ros::Duration(3).sleep();
					  picked = 0;
					  ros::Duration(1.5).sleep();
					  picked = 0;
					
					}

		
		// dropping all the cottons at end of every pose





	}
		/* Move joint4 to align with the row, so that height can be changed */
		joint_move_5.move_joint(joint5_homing_position, WAIT);//TODO This homing Position is different from Initialisation homing position but both can be same
		ros::Duration(2).sleep();
		joint_move_4.move_joint(joint4_homing_position, WAIT); // TODO move it to joint4_homing_position
                ros::Duration(2).sleep();
		joint_move_3.move_joint(joint3_homing_position, WAIT);
		ros::Duration(0.200).sleep();
		ROS_INFO("all joints are in homing_position going for l2 movement");
		//joint_move_2.move_joint(height_scan_value,WAIT);


	if(height_scan_enable ==true)
	{
		ROS_INFO("height_scan is enabled, height_scan_value:= %f", height_scan_value);
		ROS_INFO("moving to top position");
		joint_move_2.move_joint(height_scan_min,WAIT);
		ros::Duration(20).sleep();
		lid_open.command(1);
		ros::Duration(5).sleep();
		ROS_ERROR("moving to top position");
		srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
		if(joint_move::joint_idle_service.call(srv_idle)!=true)
		{
			ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
			problem_indicator_out.command(true);
			return 0;
		}


		end_time = currentTimeMillis(); 
		double time_taken = end_time - start_time; // converted into milli seconds 
		ROS_INFO("TIMETO taken by height scan is = %f ", time_taken); 
		print_timestamp(" height scan done ");
	}
}

////#####################l2 height_scan_step ##########################//
//#endif
/* Parking Position */
joint_move_5.move_joint(joint5_parking_pose, WAIT);
joint_move_4.move_joint(joint4_parking_pose, WAIT);
joint_move_3.move_joint(joint3_parking_pose, WAIT);
last_time =currentTimeMillis();



return 0;
}
