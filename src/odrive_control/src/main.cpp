#include <stdio.h>
#include <iostream>
#include <csignal>
#include <net/if.h>	
#include <sys/ioctl.h>
#include <string.h>

#include <odrive_control/task.hpp>
#include <odrive_control/misc_functions.hpp>
#include <odrive_control/debug_print.hpp>
#include <odrive_control/odrive_tests.hpp>
#include <odrive_control/odrive_can_functions.hpp>


using namespace std;

void* DUMMY_write_task(void * arg) {
 static int count  = 0 ;
 static float position = 3.0 ;
 //printf("%s invocations\n", __func__);
 controller *ControllerPtr ;
 ControllerPtr = (controller *) arg;
 if ( ++count  == 10 ) {
	count = 0 ;
   printf("%s Testing Motor canid 0 \n", __func__);
   position = position * -1 ;
   SimpleMotorTest(*ControllerPtr, 0, position );
  }
}

void* DUMMY_read_task(void * arg) {
 //printf("%s invocations\n", __func__);
 static int count  = 0 ;
 static float position = 3.0 ;
 //printf("%s invocations\n", __func__);
 controller *ControllerPtr ;
 ControllerPtr = (controller *) arg;
 if ( ++count  == 10 ) {
    count = 0 ;
   printf("%s Testing Motor canid 1 \n", __func__);
   position = position * -1 ;
   SimpleMotorTest(*ControllerPtr, 1, position );
 }
}



int maintested()
{
        int TestMotorCanID = 1 ;

	/* intialize socket can */
	std::pair<int,int> socket_array	;
	socket_array = can_init("can0" , "can0");	
	/* declare controller class object */
	controller ctrl(socket_array.second, socket_array.first);
	/* create pthread on which the RT tasks run*/
	pthread_t thread_1, thread_2;

	/* create the RT tasks */
	/* 1 : write 2: read*/
       int SCHED_P_1 = 81;
       int SCHED_P_2 = 80;
	cout << " CREATING  thread_1 with rt_task_func_1 = " << endl;
	//rt_task task_1(&thread_1, &rt_task_func_1, (void *) &ctrl, RT_PERIOD_1, SCHED_P_1);
	//rt_task write_task(&thread_1, &rt_write_task, (void *) &ctrl, RT_PERIOD_1, SCHED_P_1);
	rt_task write_task(&thread_1, &rt_write_task, (void *) &ctrl, 1000000000 , SCHED_P_1);
	cout << " created write_task" << endl;
	sleep(2) ;
	write_task.create_rt_task();
	cout << " created thread_1 with write_task " << endl;
	sleep(2) ;

	cout << " CREATING  thread_2 with read_task" << endl;
	//rt_task task_2(&thread_2, &rt_task_func_2, (void *) &ctrl, RT_PERIOD_2, SCHED_P_2);
	//rt_task read_task(&thread_2, &rt_read_task, (void *) &ctrl, RT_PERIOD_2, SCHED_P_2);
	rt_task read_task(&thread_2, &rt_read_task, (void *) &ctrl, 100000000 , SCHED_P_2);
	sleep(2) ;
	cout << " CREATING  read_task" << endl;
	read_task.create_rt_task();
	sleep(2) ;
	cout << " created thread_2 with read_task " << endl;

	/* signal handling stuff */
	/* for passing to the signit handler*/
	//tasks[0] = &write_task;
	//tasks[1] = &read_task;
	signal(SIGINT, signalHandler);


	// Reboot the AXIS	
	ctrl.reboot_odrive(0) ;
	ctrl.reboot_odrive(1) ;

	// Home the AXIS  
	// TODO : Homing of each Joints
/*
	// Initialising All the AXIS
	if (ctrl.InitialiseAxis(TestMotorCanID) != true ) { 
		printf ( " ERROR : UNABLE TO Reset Motor withCanID : %d \n", TestMotorCanID);
		exit(-1);
	}
	printf( " INFO : Motor withCanID : %d is RESET and Idle\n",TestMotorCanID );
	sleep(5) ;

	// Set the motor to be engaged
	//if( ctrl.set_axis_requested_state(TestMotorCanID,AXIS_STATE_CLOSED_LOOP_CONTROL ) != true) 
	if( ctrl.set_axis_to_state(TestMotorCanID,AXIS_STATE_CLOSED_LOOP_CONTROL ) != true) 
	{
		printf(" ERROR :NOT Able to put in CLOSED LOOP CONTROL FOR Motor withCanID : %d \n", TestMotorCanID) ;
		exit(-1);
	}
*/
	// Test each of the AXIS
        
        if (ctrl.InitialiseAxis(0)) {
	printf(" Motor withCanID : %d    is in ERROR, not able to put in  ClosedLoopControl \n",0);
        exit(-1) ;
        }
	else printf(" Motor withCanID : %d    is ClosedLoopControl \n",0);
        if (ctrl.InitialiseAxis(1)) {
	printf(" Motor withCanID : %d    is in ERROR, not able to put in  ClosedLoopControl \n",1);
        exit(-1) ;
        }
	else printf(" Motor withCanID : %d    is ClosedLoopControl \n",1);
	SimpleMotorTest(ctrl, 1);
	SimpleMotorTest(ctrl, 0);

	//float PositionEstimate = TestGetPositionEstimate( ctrl, TestMotorCanID);
	//printf("Position Estimate of MotorID : %d is : of %f\n ", TestMotorCanID, PositionEstimate) ;



	/* clean up the RT tasks*/
	//crtl.write_task.end_periodic_task();
	//read_task.end_periodic_task();
}

int maintest2(int argc, char** argv)
{
    controller TestCanBusController;
    printf("%s >> Tested the TestCanBusController\n", __func__) ;
    
    //signal(SIGSEGV, handle_segfault) ; 
    while (true) {
	    printf(" %s in the main loop \n",__func__) ;
	    sleep (1) ;
    }

}

int maintest3(int argc, char** argv)
{
    controller TestCanBusController(DUMMY_write_task, DUMMY_read_task);
    printf("%s >> Tested the TestCanBusController\n", __func__) ;
    TestCanBusController.InitialiseAxis(0);
    TestCanBusController.InitialiseAxis(1);

    //signal(SIGSEGV, handle_segfault) ; 
    while (true) {
	    printf(" %s in the main loop \n",__func__) ;
	    sleep (1) ;
            SimpleMotorTest(TestCanBusController,0);
            SimpleMotorTest(TestCanBusController,1);
    }
}
class xyz {
  public:
   controller mycontroller;
   xyz(): mycontroller() {
     mycontroller.SetReadWriteFunction(DUMMY_write_task, DUMMY_read_task) ;
     mycontroller.InitialiseAxis(0);
     mycontroller.InitialiseAxis(1);
     printf("%s, Constructor completed\n", __func__) ;
  }
};
int maintest4(int argc, char** argv)
{   xyz myxz ; 
    //controller TestCanBusController(DUMMY_write_task, DUMMY_read_task);
    //printf("%s >> Tested the TestCanBusController\n", __func__) ;
    //myxz.mycontroller.InitialiseAxis(0);
    //myxz.mycontroller.InitialiseAxis(1);

    //signal(SIGSEGV, handle_segfault) ; 
    while (true) {
	    printf(" %s in the main loop \n",__func__) ;
	    sleep (1) ;
            //SimpleMotorTest(TestCanBusController,0);
            //SimpleMotorTest(TestCanBusController,1);
    }
}


/*
int mainWithROS(int argc, char** argv)
{
    ros::init(argc, argv, "odrive_control");
    ros::NodeHandle nh;

    boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
        (new odrive_control::ODriveHWInterface(nh));

    //signal(SIGSEGV, handle_segfault) ; 

    odrive_hw_interface->init();
}
*/

int main(int argc, char** argv)
{
   maintest4(argc, argv);

}
