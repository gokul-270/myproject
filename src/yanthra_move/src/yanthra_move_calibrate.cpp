
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
//#include <yanthra_move/yanthra_move_calibrate.h>
#include <yanthra_move/yanthra_move.h>
#include <yanthra_move/joint_move.h>
#include <yanthra_move/yanthra_io.h>
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

/*
					MAIN PROGRAM
*/

int main (int argc, char** argv)
{
	ros::init(argc, argv, "yanthra_move");
	ros::NodeHandle n;

	/* Wait for node to start */
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::Rate loop_rate(100);
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
  	ROS_INFO( "Joint3 Homing Position %d ", (int)joint3_homing_position) ;

    	/* Joint 4 configuration */
	 n.param<bool>("/joint4_init/multiple_zero_poses", joint4_multiple_zero_pose, true);
	 ROS_INFO("multiple_zero_poses value read form config file := %d",joint4_multiple_zero_pose);

  	n.param<double>("/joint4_init/park_position", joint4_parking_pose, 0.001);
  	n.param<double>("joint4_init/homing_position", joint4_homing_position,0.001 );
		n.param<float>("joint4_init/theta_jerk_value",theta_jerk_value,0.0);

		n.param<float>("joint5_init/phi_jerk_value",phi_jerk_value,0.0);
		n.param<float>("l2_step_sleep_time",l2_step_sleep_time,2.0);
		n.param<float>("l2_homing_sleep_time",l2_homing_sleep_time,5.0);
  	d_out vaccum_motor(n,"vaccum_motor");



  	if(joint4_multiple_zero_pose == true)
		{
        	n.getParam("/joint4_init/zero_poses", joint4_zero_poses);
					ROS_INFO("multiple_zero_poses value read form config file := %f, %f",joint4_zero_poses[0],joint4_zero_poses[1]);
 }

  	else{
  		joint4_zero_poses.push_back(0.0);
		}

    	ROS_INFO("Joint4: Total Zero poses: %d", (int)joint4_zero_poses.size());

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
#if END_EFFECTOR_EN == true
#if AGGREGATE_PICK_EN == false
	d_out front_valve(n, "front_valve");
	d_out back_valve(n, "back_valve");
	d_out end_effector(n, "end_effector");
#elif AGGREGATE_PICK_EN == true
	a_out pick_cotton(n, "pick_cotton");
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
	a_out problem_indicator_out(n, "problem_indicator_out");
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

    if (height_scan_enable ==true) {
		srv.request.joint_id = 3;
		srv_idle.request.joint_id = 3;
	  if(joint_move::joint_homing_service.call(srv)!=true)
		{
			ROS_ERROR("Joint2 Homing, Reason: %s", srv.response.reason.c_str());
			problem_indicator_out.command(true);
			return 0;
		}

		if(joint_move::joint_idle_service.call(srv_idle)!=true)
	 {
		 ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
		 problem_indicator_out.command(true);
		 return 0;
	 }


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
//#if HEIGHT_SCAN_EN == true
//	servo_joint_2.init(); //commmented out by ribin for l2
//since the motor is initialised and homing position is done, we dont have to do the initialisation again so commenting out this init()

/*####################here we are gonna add the call for l2 to initialised##################*/

/*##########################initialisation of l2 compleated################################*/

	//ros::Duration(10.0).sleep();
/*	do
	{
	    ros::Duration(2.0).sleep();
#endif */
int number_of_steps =0;

if (height_scan_enable ==true) {
  number_of_steps = height_scan_max /height_scan_step;
} // 10/2 =5 //0.4 /0.15
else{
	 number_of_steps =0;
}
double height_scan_value = height_scan_min;

for(int i=0; i<=number_of_steps; i++ )
{


	if (height_scan_enable ==true) {
		if (height_scan_value < height_scan_max)
		{
		joint_move_2.move_joint(height_scan_value,WAIT);
		ros::Duration(l2_step_sleep_time).sleep();
		ROS_INFO("height_scan_value %f:= ",height_scan_value);
		height_scan_value = height_scan_value+height_scan_step;
		ROS_INFO("height_scan_step value %f:=", height_scan_step);
		srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
		if(joint_move::joint_idle_service.call(srv_idle)!=true)
		{
		 ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
		 problem_indicator_out.command(true);
		 return 0;
	 }
//		ROS_INFO("height_scan_value %f:=", height_scan_value);

	}

}

            for(size_t joint4_cnt = 0; joint4_cnt < joint4_zero_poses.size(); joint4_cnt++)
            {
                ROS_INFO("Joint4: Moving to Zero Pose: %f", (int)joint4_cnt);

                /* Make link3 to look down, while capturing */

                ROS_INFO("Joint3: Moving to Homing Position: \n");
                joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                joint_move_3.move_joint(joint3_homing_position, WAIT); //TODO change 0.001 to joint_zero_poses[joint3_cnt]
                joint_move_4.move_joint(joint4_zero_poses[joint4_cnt], WAIT);

                /* Initiate a camera start request */
#if CAMERA_EN == true
                led_control.command(START);
		//vaccum_motor.command(START); //added by ribin for controlling the blower motor
                ROS_INFO("Executing: " COTTON_DETECT_PROGRAM " " YANTHRA_DATA_INPUT_DIR "/ " YANTHRA_DATA_OUTPUT_DIR "/");
                int state = system(COTTON_DETECT_PROGRAM " " YANTHRA_DATA_INPUT_DIR "/ " YANTHRA_DATA_OUTPUT_DIR "/");
                if(state != 0)
                {
                    ROS_ERROR("Cotton Detection Failed: %d", state);
                    problem_indicator_out.command(true);
                }
                led_control.command(STOP);
                record_debug_data();		// Store the captured images in debug folder
#endif
                /* Get the coordinates of cotton */
                ROS_INFO("Read the coordinates");
                get_cotton_coordinates(&positions);

                /* Convert cotton coordinates from camera_frame to link3 frame */
#if CAMERA_EN == true
                getCottonCoordinates_cameraToLink3(&listener_camera_base, &positions, &positions_link3);
#else
                getCottonCoordinates_ToLink3(&positions, &positions_link3);
#endif






                int trgt = 0;
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
                    ROS_WARN( "In yanthra_move_calibrate.cpp") ;
                    ROS_WARN( " YANTHRA_MOVE found R : %f, theta : %f , phi :%f ",r, theta, phi) ;
                    ROS_ERROR( "CHECKING REACHABILITY") ;
                    if(check_reachability(r, theta, phi) == true)
                    {
											if(Global_vaccum_motor == true)
											{
												vaccum_motor.command(START); //added by ribin for controlling the blower motor
												//TODO check whether the motor is still on or not
												}

                        joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]

                        joint_move_4.move_joint(joint4_zero_poses[joint4_cnt] + theta, WAIT);
                        joint_move_3.move_joint(joint3_homing_position + phi, WAIT);
												//here we are gonna add a jerk to the theta



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

                        joint_move_5.move_joint(r, WAIT);
												if(jerk_enabled_theta == true)
												{
													float delta_theta = theta_jerk_value/r;
													ROS_INFO("delta_theta %lf",delta_theta);
													joint_move_4.move_joint(joint4_zero_poses[joint4_cnt] + theta + delta_theta, WAIT);
													joint_move_4.move_joint(joint4_zero_poses[joint4_cnt] + theta - delta_theta, WAIT);
													joint_move_4.move_joint(joint4_zero_poses[joint4_cnt] + theta, WAIT);

												}
												if(jerk_enabled_phi == true)
												{
													float delta_phi =phi_jerk_value/r;
													ROS_INFO("delta_phi %lf",delta_phi);
													joint_move_3.move_joint(joint3_homing_position + phi + delta_phi, WAIT);
													joint_move_3.move_joint(joint3_homing_position + phi - delta_phi , WAIT);
													joint_move_3.move_joint(joint3_homing_position + phi, WAIT);

												}

                    /*    // ros::Duration(1).sleep();
                        joint_move_5.move_joint(r-0.025, WAIT);     //TODO Jerky Motion //Author Mani
                        // ros::Duration(1).sleep();
                        joint_move_5.move_joint(r+0.010, WAIT); */
                        ros::Duration(picking_delay).sleep();
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
                } // for ((std::vector<geometry_msgs::PointStamped>::iterator it = positions_link3.begin(); it != positions_link3.end(); it++
								if(Global_vaccum_motor == true){
										vaccum_motor.command(STOP);	//here we are gonna publish a msg that gonna swithoff the blower
								}
		} // for(size_t joint4_cnt = 0; joint4_cnt < joint4_zero_poses.size(); joint4_cnt++)
//#if HEIGHT_SCAN_EN == true


            /* Move joint4 to align with the row, so that height can be changed */
            joint_move_5.move_joint(joint5_homing_position, WAIT);//TODO This homing Position is different from Initialisation homing position but both can be same
            ros::Duration(0.1).sleep();
            joint_move_4.move_joint(joint4_homing_position, WAIT); // TODO move it to joint4_homing_position
            joint_move_3.move_joint(joint3_homing_position, WAIT);
						ros::Duration(0.200).sleep();
						ROS_INFO("all joints are in homing_position going for l2 movement");
						//joint_move_2.move_joint(height_scan_value,WAIT);



/*
if (height_scan_enable ==true) {

	if (height_scan_value < height_scan_max)
	{
	joint_move_2.move_joint(height_scan_value,WAIT);
	ROS_INFO("height_scan_value %d:= ",height_scan_value);
	height_scan_value = height_scan_value+height_scan_step;
	ROS_INFO("height_scan_step value %d :=", height_scan_step);
	ROS_INFO("height_scan_max value %d :=", height_scan_max);

}
	if (height_scan_value >= height_scan_max)
	{
		//height_scan_value = height_scan_step; logic error .0.2
		height_scan_value = height_scan_min;
		ROS_INFO("height_scan_value %f:= ",height_scan_value);

	}
}
*/

}
if(height_scan_enable ==true)
{
		 ROS_INFO("height_scan is enabled, height_scan_value:= %f", height_scan_value);
		 ROS_INFO("moving to top position");
		 joint_move_2.move_joint(height_scan_min,WAIT);
		 ros::Duration(l2_homing_sleep_time).sleep();
		 srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
		 if(joint_move::joint_idle_service.call(srv_idle)!=true)
		{
		 ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
		 problem_indicator_out.command(true);
		 return 0;
		}

}

}

	//while(servo_joint_2.move(height_scan_step)); //commmented out by ribin //this is the code used for moving the servo motor step by step,
//so we are gonna change this code for calling odrive so we can move step by step
//##########################l2 height_scan_step ###############//
//while (true) {

//break;
//}
//here we have to pass the value for moving the l2 up and down, repeatedly so lets assume the l2 is at homing_position
//so the starting posiiton is 0.0 , the max pos and min pose is loaded from the config file.
//lets consider the min pose to be 0.0 and max position to be 1000 cm. or 1 meter
//here we have to find the zero_offset_
////#####################l2 height_scan_step ##########################//
//#endif
		/* Parking Position */
		joint_move_5.move_joint(joint5_parking_pose, WAIT);
		joint_move_4.move_joint(joint4_parking_pose, WAIT);
		joint_move_3.move_joint(joint3_parking_pose, WAIT);




	return 0;
}
