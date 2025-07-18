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


float TestGetPositionEstimate( controller ctrl, int CanID)
{
  ctrl.RequestMsg.get_encoder_estimate(CanID);
  //sleep(1) ;// WaitForReadCycleTime() ;
  //long TimeInMilliSeconds = CurrentTimeInMillisec();
  //printf("TestGetPositionEstimate >> called RequestMsg.get_encoder_estimate at TIME  %ld with \n",TimeInMilliSeconds ) ;
    
  odrive_motor* MotorData = ctrl.get_motor_data(CanID);
  return MotorData->encoder_pos_estimate;
}


#define DEBUG_MOVE true
#define DEBUG_READ false
#define READ_AndPrint_POSITIONESTIMATE  \
	if (DEBUG_READ) { \
        printf("**Reading Position Estimate of MotorID : %d \n ", CanID) ;\
	float PositionEstimate = ctrl.GetPositionEstimate(  CanID);\
	printf("**Position Estimate of MotorID : %d is : of %f\n ", CanID, PositionEstimate) ; }

void SimpleMotorTest( controller ctrl, int CanID, float Position = 3.0 )
{  
    //static float Position = 1.0 ; 
    //float PositionEstimate; // Unused Variable
    // Simple testing
    //ctrl.InitialiseAxis(CanID) ;
    Position = Position * (-1) ;
    READ_AndPrint_POSITIONESTIMATE  ;
    ctrl.set_pos_setpoint(CanID, Position , 0.0 , 0.0 ) ;
/*
    for (int i= 0 ; i < 5; i++ ) {
      sleep(0.1) ;
      //printf("Iteration %d\n",i) ;
      READ_AndPrint_POSITIONESTIMATE  ;
    }

    ctrl.set_pos_setpoint(CanID , 11.0, 0.0, 0.0  ) ;
    sleep(1) ;
    cout << " Motor withCanID :" << CanID << " is Moving 11 rotations ^^^ " << endl ;
    for (int i= 0 ; i < 5; i++ ) {
      sleep(0.5) ;
      //printf("Iteration %d\n",i) ;
      READ_AndPrint_POSITIONESTIMATE  ;
    }
    sleep(1) ;
    READ_AndPrint_POSITIONESTIMATE  ;


    ctrl.set_pos_setpoint(CanID, -6.0, 0.0 , 0.0 ) ;
    cout << " Motor withCanID :" << CanID << " is Moving -6 rotations ^^^" << endl ;

    sleep(1) ;
    for (int i= 0 ; i < 5; i++ ) {
      sleep(0.5) ;
      //printf("Iteration %d\n",i) ;
      READ_AndPrint_POSITIONESTIMATE  ;
    }
    sleep(1) ;
    READ_AndPrint_POSITIONESTIMATE  ;
    sleep(1) ;
    READ_AndPrint_POSITIONESTIMATE  ;

    ctrl.set_pos_setpoint(CanID, 0.0, 0.0 , 0.0 ) ;
    cout << " Motor withCanID :" << CanID << " is Moving to 0 position ^^^^" << endl ;
    sleep(1) ;
    for (int i= 0 ; i < 5; i++ ) {
      sleep(0.5) ;
      printf("Iteration %d\n",i) ;
      READ_AndPrint_POSITIONESTIMATE  ;
    }
    sleep(1) ;
    READ_AndPrint_POSITIONESTIMATE  ;
  */
}


void InitialiseAllAxis(controller ctrl)
{	// Reboot the AXIS	
	ctrl.reboot_odrive(0) ;
        int num_joints_ = 2 ;
	// Initialising All the AXIS
	for (int canID = 0 ; canID < num_joints_ ; canID++ )
	{
		if (ctrl.InitialiseAxis(canID) != true ) { 
			printf ( " ERROR : UNABLE TO Reset Motor withCanID : %d \n", canID);
			exit(-1);
		}
		printf( " INFO : Motor withCanID : %d is RESET and Idle\n",canID);
		sleep(5) ;

		// Set the motor to be engaged
		//if( ctrl.set_axis_requested_state(TestMotorCanID,AXIS_STATE_CLOSED_LOOP_CONTROL ) != true) 
		if( ctrl.set_axis_to_state(canID,AXIS_STATE_CLOSED_LOOP_CONTROL ) != true) 
		{
		printf(" ERROR :NOT Able to put in CLOSED LOOP CONTROL FOR Motor withCanID : %d \n", canID) ;
		exit(-1);
		}
	}
}
/*
int CleanUpCanBusInterface()
{
    // clean up the RT tasks
	write_task.end_periodic_task();
	read_task.end_periodic_task();
}
*/


