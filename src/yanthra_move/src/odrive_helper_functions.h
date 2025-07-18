/*

   This part of the code is culled out from odrive_control.cpp
   it has only the ODriveHWInterface object so it can be directly 
   accessed from yanthra_move
   Other functions like the Async Spinner has been removed

   THIS IS TO REMOVE THE SCHEDULING OF another node and publish / subscribe queue which seems to be in deadlock
*/

#include <odrive_control/generic_hw_control_loop.h>
#include <odrive_control/odrive_hw_interface.h>

{

    boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
        (new odrive_control::ODriveHWInterface(nh));
    odrive_hw_interface->init();
    odrive_hw_interface->read(elapsed_time );
    odrive_hw_interface->write(elapsed_time );
    ROS_DEBUG("odrive_control:main() >> Completed an iteration of odrive_control while loop");
    rate.sleep();
}

