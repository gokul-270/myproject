#include<stdio.h>
#include <iostream>
#include <csignal>
#include <net/if.h>	
#include <sys/ioctl.h>
#include <string.h>
#include <odrive_control/task.hpp>
#include <odrive_control/misc_functions.hpp>
#include <odrive_control/odrive_can_functions.hpp>
#include <odrive_control/debug_print.hpp>
using namespace std;


float TestGetPositionEstimate( controller ctrl, int CanID);

void SimpleMotorTest( controller ctrl, int CanID, float position = 3.0);

void InitialiseCanInterface(controller ctrl, int TestMotorCanID) ;
void InitialiseAllAxis(controller ctrl);

/*
int CleanUpCanBusInterface()
{
    // clean up the RT tasks
	write_task.end_periodic_task();
	read_task.end_periodic_task();
}
*/


