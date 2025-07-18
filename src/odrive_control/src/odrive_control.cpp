
/*  Author: Saurabh Bansal
Desc:   O-Drive Hardware control loop,
To recieve commands from topics and control
the robot using odrive-hardware
 */
#include <stdio.h>
// #include "signal_generator.hpp"
#include <iostream>
#include <csignal>
#include <net/if.h>	
#include <sys/ioctl.h>
#include <string.h>
#include <algorithm>
#include <odrive_control/misc_functions.hpp>
#include <odrive_control/debug_print.hpp>
#include <odrive_control/task.hpp>
#include <odrive_control/generic_hw_control_loop.h>
#include <odrive_control/odrive_hw_interface.h>
#include <odrive_control/odrive_can_functions.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odrive_control");
    ros::NodeHandle nh;

    signal(SIGSEGV, handle_segfault) ; 
    boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
        (new odrive_control::ODriveHWInterface(nh));
    //controller CanBusController ;
    //CanBusController.SetReadWriteFunction(DUMMY_write_task, DUMMY_read_task);

    //odrive_hw_interface->init();
    // Commenting out this to remove the control_loop : 20FEB2023 Manohar Sambandam   
    /*
       ros_control_boilerplate::GenericHWControlLoop control_loop(nh, odrive_hw_interface);

       control_loop.run(); // Blocks untill shutdown signal is recieved
     */
    ROS_INFO(" %s is just going through indefinite sleep loop", __func__);
    while (true ) {
	   printf("%s going to sleep 1 second \n",__func__) ;
	   sleep(1.0);
    }

    ROS_INFO(" Initiating Ros Spin \n");
    ros::AsyncSpinner spinner(0); // Changed from zero (which is equal to number of CPUs) to just 1 as experiment
    // The control_manager which runs the joint3/4/5 controllers are failing now
    spinner.start();
    ROS_INFO(" Ros Spin Started\n");

    double loop_hz_ = 10;
    struct timespec current_time ;
    struct timespec last_time ;
    ros::Duration elapsed_time ; 
    ros::Rate rate(loop_hz_);
    // Get current time for use with first update                                         
    clock_gettime(CLOCK_MONOTONIC, &last_time);       
    // Adding the while loop for reading and writing to the odrive motors
    //odrive_hw_interface->read(elapsed_time );
    //odrive_hw_interface->write(elapsed_time );
    while(ros::ok()) {
        ROS_INFO("%s Inside While Ros::ok() function\n", __func__);
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        elapsed_time =                                                                                                                       
            ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec) / ros_control_boilerplate::BILLION);
        last_time = current_time;
        // ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "generic_hw_main","Sampled update loop with elapsed
        // time " << elapsed_time_.toSec());

        // Error check cycle time
        /*
           const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
           if (cycle_time_error > cycle_time_error_threshold_)
           {
           ROS_WARN_STREAM_NAMED(name_, "GenericHWControlLoop::update() Cycle time exceeded error threshold by: "
           << cycle_time_error << ", cycle time: " << elapsed_time_
           << ", threshold: " << cycle_time_error_threshold_);
           }
         */

        // Will call the read and Write loops here.
        //update();
        //odrive_hw_interface->read(elapsed_time );
        //odrive_hw_interface->write(elapsed_time );
        ROS_INFO("%s >> odrive_control:main() >> Completed an iteration of odrive_control while loop",__func__);
        rate.sleep();
    }
    ROS_INFO("** Exiting odrive_control **");
    return 0;
}
