//   Last Updated : 4DEC2023
//   Last Compiled : 
//   History :
/* Created by Sumantra on 9th December 2019 */
/* Ver 1.1 Major bug fix by Chang Hong 10 Dec 2019 */
//  04DEC2023 - Modified the GetPositionEstimate to return the value of the joint which is updated by the ::read()
//  04DEC2023 - Modified ProcessData to read the Encoder Estimate also. Originally GetPosition Estimate was reading this but is now
//              done in the read loop. The write loop will also make a  CANBUS remote call for reading all the joints estimate. 
//  24JUN2023 - Added GetPositionEstimate 
//  27JUN2023 created functions to initialise controller class with
//        default can interface name  "can0"
//                read_rt and write_rt tasks running in 2 independent threads 
// 
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
#include <odrive_control/odrive_can_functions.hpp>
#include "odrive_tests.cpp"

/*MSG HANDLER FUNCTIONS FOR PROCESSING AND ORGANIZING INCOMING MSGS*/
//define command ids

#define ODRIVE_HEARTBEAT        1
#define ESTOP                   2
#define GET_MOTOR_ERROR         3  // IMPLEMENTED IN REQUEST_MSG
#define GET_ENCODER_ERROR       4  // IMPLEMENTED IN REQUEST_MSG
#define SET_AXIS_NODE_ID        6
#define SET_AXIS_REQUESTED_STATE  7
#define GET_ENCODER_ESTIMATE    9 // IMPLEMENTED IN REQUEST_MSG 
#define GET_ENCODER_COUNTS      10 // IMPLEMENTED IN REQUEST_MSG 
#define MOVE_TO_POS             11
#define SET_POS_SETPOINT        12
#define SET_VEL_SETPOINT        13
#define SET_CUR_SETPOINT        14
#define SET_VEL_LIMIT           15
#define START_ANTI_COGGING      16
#define SET_TRAJ_VEL_LIMIT      17
#define SET_TRAJ_ACCEL_LIMIT    18
#define SET_TRAJ_A_PER_CSS      19
#define GET_IQ_VALUES           20 // IMPLEMENTED IN REQUEST_MSG 
#define REBOOT_ODRIVE           22
#define GET_VBUS_VOLTAGE        23  // IMPLEMENTED IN REQUEST_MSG
#define SET_VEL_PI_GAIN         24

#define HEARTBEATTIME 0.002 // hearbeat time is 100ms but made is 200ms for processing 
#define ENCODERINDEXSEARCHTIME 4
#define CHANGEOFSTATETIME 1
#define WAITFORCHANGEOFSTATE() sleep(CHANGEOFSTATETIME) 
#define WAITFORENCODEINDEXSEARCH() sleep(ENCODERINDEXSEARCHTIME) 
#define CANBUSTIMEOUTVALUE 3250 // 3250 milliseconds

/*CAN READ AND WRITE FUNCTIONS*/

struct can_Message_t {
    uint32_t id = 0x000;  // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    bool isExt = false;
    bool rtr = false;
    uint8_t len = 8;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
} ;

struct can_Signal_t {
    const uint8_t startBit;
    const uint8_t length;
    const bool isIntel;
    const float factor;
    const float offset;
};

///*
//struct can_frame {
//         canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
//         __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
//         __u8    __pad;   /* padding */
//         __u8    __res0;  /* reserved / padding */
//         __u8    __res1;  /* reserved / padding */
//         __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
// };
//*/

void InitialiseCanAxis( controller ctrl, int CanID)
{   
    // Simple testing
    ctrl.InitialiseAxis(CanID) ;
}

void CopyFromCanMsgToCanSignal(can_Message_t &CanMsg, struct can_frame &CanFrame)
{
   CanMsg.id =  CanFrame.can_id;
   CanMsg.len = CanFrame.can_dlc;
   CanMsg.buf[0] = CanFrame.data[0]  ;
   CanMsg.buf[1] = CanFrame.data[1]  ;
   CanMsg.buf[2] = CanFrame.data[2]  ;
   CanMsg.buf[3] = CanFrame.data[3]  ;
   CanMsg.buf[4] = CanFrame.data[4]  ;
   CanMsg.buf[5] = CanFrame.data[5]  ;
   CanMsg.buf[6] = CanFrame.data[6]  ;
   CanMsg.buf[7] = CanFrame.data[7]  ;
}

#include <iterator>

// Extraction of position & Velocity from can_getSignal function
// .//ODriveFlexCAN/src/ODriveCanbusTranslator.h:
/*
GetEncoderEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
GetEncoderEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
Heartbeat.error = (AxisError)can_getSignal<uint32_t>(msg, 0, 32, true);
Heartbeat.state = (AxisState)can_getSignal<uint32_t>(msg, 32, 32, true);
GetMotorError.error = (MotorError)can_getSignal<uint32_t>(msg, 0, 32, true); // or 64?
GetEncoderError.error = (EncoderError)can_getSignal<uint32_t>(msg, 0, 32, true);
GetSensorlessError.error = (SensorlessEstimatorError)can_getSignal<uint32_t>(msg, 0, 32, true);
GetEncoderEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
GetEncoderEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
GetEncoderCount.shadow_count = can_getSignal<int32_t>(msg, 0, 32, true, 1, 0);
GetEncoderCount.count_cpr = can_getSignal<int32_t>(msg, 32, 32, true, 1, 0);
GetIQ.iq_setpoint = can_getSignal<float>(msg, 0, 32, true, 1, 0);
GetIQ.iq_measured = can_getSignal<float>(msg, 32, 32, true, 1, 0);
GetSensorlessEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
GetSensorlessEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
GetVbusVoltage.vbus = can_getSignal<float>(msg, 0, 32, true, 1, 0);
*/

// This will initialise the Can Interface for can0
// create both rx and tx tasks and run them as independent threads

rt_task * tasks[2];

void signalHandler( int signum ) 
{
   printf("%s Interrupt signal %ld received \n", __func__,  signum );
   //write_task.set_signal_handler(false);
   //read_task.set_signal_handler(false);
   exit(signum);  
}	

std::pair <int,int> can_init(const char* read_port, const char*  write_port)

{
    int socket_read, socket_write;
    struct sockaddr_can addr_read, addr_write;
    struct ifreq ifr_read, ifr_write;
    int NoOfMotors = 3;
    struct can_filter rtr_filter[NoOfMotors]; // NoOfMotors is set to 6
    rtr_filter[0].can_id = (uint32_t) (1 << 30) ^ (0xFFFFFFFF);// NO frames with RTR bit 
    //this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
    //rtr_filter(0).can_id = (0x0) | (1<<30) ;// Setting the RTR flag
    rtr_filter[0].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    //rtr_filter[0].can_mask = (CAN_ERR_FLAG| CAN_RTR_FLAG | CAN_SFF_MASK);
    //rtr_filter[0].can_mask = CAN_SFF_MASK;
    rtr_filter[1].can_id = (uint32_t) (1 << 30) ^ (0xFFFFFFFF);// NO frames with RTR bit 
    rtr_filter[0].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    //rtr_filter[0].can_mask = (CAN_ERR_FLAG| CAN_RTR_FLAG | CAN_SFF_MASK);
    //rtr_filter[1].can_mask = CAN_SFF_MASK;
   /*
    rtr_filter[2].can_id = (0x2 );
    //rtr_filter[0].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    //rtr_filter[0].can_mask = (CAN_ERR_FLAG| CAN_RTR_FLAG | CAN_SFF_MASK);
    rtr_filter[2].can_mask = CAN_SFF_MASK;
    rtr_filter[1].can_id = (0x1);
    rtr_filter[1].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    rtr_filter[2].can_id = (0x2);
    rtr_filter[2].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    rtr_filter[3].can_id = (0x3);
    rtr_filter[3].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    rtr_filter[4].can_id = (0x4);
    rtr_filter[4].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
    rtr_filter[5].can_id = (0x5);
    rtr_filter[5].can_mask = (CAN_RTR_FLAG | CAN_SFF_MASK);
   */

    socket_write = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    socket_read = socket(PF_CAN, SOCK_RAW, CAN_RAW);
   
    if (setsockopt(socket_read, SOL_CAN_RAW, CAN_RAW_FILTER, &rtr_filter, sizeof(rtr_filter)) ){
         printf("ERROR in setsockoption from can_init()\n");
         //printf(errno); 
         //perror();
         exit(-1) ;
    }

    strcpy(ifr_read.ifr_name, read_port );
    ioctl(socket_read, SIOCGIFINDEX, &ifr_read);

    strcpy(ifr_write.ifr_name, write_port);
    ioctl(socket_write, SIOCGIFINDEX, &ifr_write);

    addr_read.can_family = AF_CAN;
    addr_read.can_ifindex = ifr_read.ifr_ifindex;

    addr_write.can_family = AF_CAN;
    addr_write.can_ifindex = ifr_write.ifr_ifindex;

    bind(socket_read, (struct sockaddr *)&addr_read, sizeof(addr_read));
    bind(socket_write, (struct sockaddr *)&addr_write, sizeof(addr_write));
    
    std::pair<int,int> pair(socket_read,socket_write);
    return pair;
}


#define DEBUG_THREAD true 
#define DEBUG_WRITE  true 
#define DEBUG_MOVE true
#define DEBUG_READ false

#define READ_AndPrint_POSITIONESTIMATE  \
	if (DEBUG_READ) { \
        printf("**Reading Position Estimate of MotorID : %d \n ", CanID) ;\
	float PositionEstimate = ctrl.GetPositionEstimate(  CanID);\
	printf("**Position Estimate of MotorID : %d is : of %f\n ", CanID, PositionEstimate) ; }

static bool start_write = true;  
float  POSITION = 3 ;

void* MY_write_task(void * arg) {
 printf("%s invocations\n", __func__);
}

void* MY_read_task(void * arg) {
 printf("%s invocations\n", __func__);
}

// thread 1 : write data on CAN BUS //
//void* rt_task_func_1(void * arg)
void* rt_write_task(void * arg)
{
	if ( DEBUG_THREAD) { 
		if (start_write){ 
		        printf("%s ", __func__) ; 
			PrintThreadID() ;
		}
                start_write= false;
	}

        uint32_t TestMotorCanID = 1 ; 
        static long PreviousScheduledTimeInMilliSeconds = 0;
        //long PreviousScheduledTimeInMilliSeconds = 0;
        long PeriodInMilliSeconds = 0;
        long TimeInMilliSeconds = CurrentTimeInMillisec();
        PeriodInMilliSeconds = TimeInMilliSeconds - PreviousScheduledTimeInMilliSeconds ;
        PreviousScheduledTimeInMilliSeconds = TimeInMilliSeconds ;
        if (DEBUG_WRITE) 
	printf("%s Entering rt_write_task at %ld with a period of %ld\n ", __func__, TimeInMilliSeconds, PeriodInMilliSeconds);
	controller * ctrl = (controller *) arg;
        POSITION = POSITION * - 1;
        if (DEBUG_WRITE) {
        float PositionEstimate0 = ctrl->GetPositionEstimate(0) ;
        float PositionEstimate1 = ctrl->GetPositionEstimate(0) ;
	printf("Position Estimate of MotorID : %d is :  %f\n ", 0, PositionEstimate0) ;
	printf("Position Estimate of MotorID : %d is :  %f\n ", 1, PositionEstimate1) ;
	printf("Setting Position of  MotorID : %d To :  %f\n ",0, POSITION) ;
	printf("Setting Position of  MotorID : %d To :  %f\n ",1, POSITION) ;
	}
        ctrl->set_pos_setpoint(0, POSITION, 0.0, 0.0) ;
        ctrl->set_pos_setpoint(1 , POSITION, 0.0, 0.0) ;
        sleep(2) ;	
        if (DEBUG_WRITE) {
        float PositionEstimate0 = ctrl->GetPositionEstimate(0) ;
        float PositionEstimate1 = ctrl->GetPositionEstimate(0) ;
	printf("Position Estimate of MotorID : %d is :  %f\n ", 0, PositionEstimate0) ;
	printf("Position Estimate of MotorID : %d is :  %f\n ", 1, PositionEstimate1) ;
	}
	
}

/* thread 2 : read data from CAN BUS*/
static bool start_read = true;  
void* rt_read_task(void * arg)
{
	if ( DEBUG_THREAD) { 
		if (start_read) {
		        printf("%s ", __func__) ; 
			PrintThreadID() ;
                }
              start_read = false;
	}


        static long Read_PreviousScheduledTimeInMilliSeconds = 0;
        long PeriodInMilliSeconds = 0;
        long TimeInMilliSeconds = CurrentTimeInMillisec();
        PeriodInMilliSeconds = TimeInMilliSeconds - Read_PreviousScheduledTimeInMilliSeconds ;
        Read_PreviousScheduledTimeInMilliSeconds = TimeInMilliSeconds ;
        if (DEBUG_READ) 
	printf("%s Entering rt_read_task at %ld with a period of %ld\n ", __func__, TimeInMilliSeconds, PeriodInMilliSeconds);
	
	controller * ctrl = (controller *) arg;
	ctrl->msg_handler();
}

// defining a new constructor where the real time functions are passed 
//controller::controller(void* WriteFunctionPtr(void *), void* rt_read_task(void*) )

//                CanBusController.SetReadWriteFunction(std::function<void*(void*)> writeexp, std::function<void*(void*)> readexp) ;
//void controller::SetReadWriteFunction(std::function<void*(void*)> WriteFunctionPtr, std::function<void*(void*)> rt_read_task)
void controller::SetReadWriteFunction(void* WriteFunctionPtr(void *),void*  rt_read_task(void *) )
//void controller::SetReadWriteFunction(void* WriteFunctionPtr(void *), void* rt_read_task(void*)) 
{
    printf("%s Calling the constructor for controller()\n", __func__);

	// intialize socket cand
	std::pair<int,int> socket_array	;
	socket_array = can_init("can0" , "can0");	

        int writesocket_fd = socket_array.first ;
	int readsocket_fd  = socket_array.second;

	//initialize rx and tx can messages structs
	for (int i = 0; i < 8; ++i)
	{	
		this->tx_msg.cframe.data[i] = 0;
		this->rx_msg.cframe.data[i] = 0;
	}
	this->tx_msg.cframe.can_dlc = 8; 
	this->rx_msg.cframe.can_dlc = 8; 
	this->write_socket = writesocket_fd;
	this->read_socket = readsocket_fd;
	printf("read socket: %d, write socket: %d\n",read_socket,write_socket );

    RequestMsg.set_socket(writesocket_fd) ;

       this->reboot_odrive(0);
       this->reboot_odrive(1);
	// Test each of the AXIS
        InitialiseCanAxis(*this, 0);
        InitialiseCanAxis(*this, 1);
	//SimpleMotorTest(*this, 1);
	//SimpleMotorTest(*this, 0);


    // Create threads for the Read and Write Tasks with appropriate priority
    int SCHED_P_2 = 80;
    int SCHED_P_1 = 81 ;
    int PeriodInNanoSec = 1000000000 ;
	// create pthread on which the RT tasks run //
	pthread_t thread_1, thread_2;

	printf( " %s >> CREATING  rt_task write_task \n", __func__) ;
    printf( " %s >> write task running with a period of : 1000000000 ns \n", __func__) ;
	//rt_task task_1(&thread_1, &rt_task_func_1, (void *) &ctrl, RT_PERIOD_1, SCHED_P_1);
	//rt_task write_task(&thread_1, &rt_write_task, (void *) this , 1000000000 , SCHED_P_1);
	rt_task write_task(&thread_1, WriteFunctionPtr, (void *) this , PeriodInNanoSec  , SCHED_P_1);
	printf( " %s >> created  rt_task write_task \n", __func__) ;
	//sleep(2) ;	
	printf( " %s >>  CREATING  write_task.create_rt_task \n", __func__) ;
	write_task.create_rt_task();
	printf(" %s > Completed write_task.create_rt_task \n", __func__) ;
	//sleep(2) ;

	printf( " %s >> CREATING  thread_2 with read_task = \n", __func__) ;
    printf( " %s >> read_task running with a period of : 1000000000 ns \n", __func__) ;
	//rt_task task_2(&thread_2, &rt_task_func_2, (void *) &ctrl, RT_PERIOD_2, SCHED_P_2);
	rt_task read_task(&thread_2, rt_read_task, (void *) this, PeriodInNanoSec  , SCHED_P_2);
	//rt_task read_task(&thread_2, &rt_read_task, (void *) &ctrl, 100000000 , SCHED_P_2);
	//sleep(2) ;
	printf( " %s >> created  rt_task read_task \n", __func__) ;
	printf( " %s >>  CREATING  read_task.create_rt_task \n", __func__) ;
	read_task.create_rt_task();
	//sleep(2) ;
	printf( " %s >>  Completed read_task.create_rt_task \n", __func__) ;
	signal(SIGINT, signalHandler);
	signal(SIGSEGV, handle_segfault);
}

controller::controller()
{
        printf("%s Calling the constructor for controller()\n", __func__);

	// intialize socket ca
	std::pair<int,int> socket_array	;
	socket_array = can_init("can0" , "can0");	

        int writesocket_fd = socket_array.first ;
	int readsocket_fd  = socket_array.second;

	//initialize rx and tx can messages structs
	for (int i = 0; i < 8; ++i)
	{	
		this->tx_msg.cframe.data[i] = 0;
		this->rx_msg.cframe.data[i] = 0;
	}
	this->tx_msg.cframe.can_dlc = 8; 
	this->rx_msg.cframe.can_dlc = 8; 
	this->write_socket = writesocket_fd;
	this->read_socket = readsocket_fd;
	printf("read socket: %d, write socket: %d\n",read_socket,write_socket );

        RequestMsg.set_socket(writesocket_fd) ;
}

/*
controller::controller()
{
        printf("%s Calling the constructor for controller()\n", __func__);

	// intialize socket ca
	std::pair<int,int> socket_array	;
	socket_array = can_init("can0" , "can0");	

    int writesocket_fd = socket_array.first ;
	int readsocket_fd  = socket_array.second;

	//initialize rx and tx can messages structs
	for (int i = 0; i < 8; ++i)
	{	
		this->tx_msg.cframe.data[i] = 0;
		this->rx_msg.cframe.data[i] = 0;
	}
	this->tx_msg.cframe.can_dlc = 8; 
	this->rx_msg.cframe.can_dlc = 8; 
	this->write_socket = writesocket_fd;
	this->read_socket = readsocket_fd;
	printf("read socket: %d, write socket: %d\n",read_socket,write_socket );

    RequestMsg.set_socket(writesocket_fd) ;

    // Create threads for the Read and Write Tasks with appropriate priority
    int SCHED_P_2 = 80;
    int SCHED_P_1 = 81 ;
	// create pthread on which the RT tasks run //
	pthread_t thread_1, thread_2;

	printf( " %s >> CREATING  rt_task write_task \n", __func__) ;
    printf( " %s >> write task running with a period of : 1000000000 ns \n", __func__) ;
	//rt_task task_1(&thread_1, &rt_task_func_1, (void *) &ctrl, RT_PERIOD_1, SCHED_P_1);
	rt_task write_task(&thread_1, &rt_write_task, (void *) this , 1000000000 , SCHED_P_1);
	printf( " %s >> created  rt_task write_task \n", __func__) ;
	sleep(2) ;	
	printf( " %s >>  CREATING  write_task.create_rt_task \n", __func__) ;
	write_task.create_rt_task();
	printf(" %s > Completed write_task.create_rt_task \n", __func__) ;
	sleep(2) ;

	printf( " %s >> CREATING  thread_2 with read_task = \n", __func__) ;
    printf( " %s >> read_task running with a period of : 1000000000 ns \n", __func__) ;
	//rt_task task_2(&thread_2, &rt_task_func_2, (void *) &ctrl, RT_PERIOD_2, SCHED_P_2);
	rt_task read_task(&thread_2, &rt_read_task, (void *) this, 1000000000 , SCHED_P_2);
	//rt_task read_task(&thread_2, &rt_read_task, (void *) &ctrl, 100000000 , SCHED_P_2);
	sleep(2) ;
	printf( " %s >> created  rt_task read_task \n", __func__) ;
	printf( " %s >>  CREATING  read_task.create_rt_task \n", __func__) ;
	read_task.create_rt_task();
	sleep(2) ;
	printf( " %s >>  Completed read_task.create_rt_task \n", __func__) ;

       this->reboot_odrive(0);
       this->reboot_odrive(1);
	// Test each of the AXIS

        InitialiseCanAxis(*this, 0);
        InitialiseCanAxis(*this, 1);
	//SimpleMotorTest(*this, 1);
	//SimpleMotorTest(*this, 0);

	signal(SIGINT, signalHandler);
	signal(SIGSEGV, handle_segfault);
}
*/

/*
controller::controller()
{
  printf("This is a dummy constructor for %s\n", __func__);

  //this->SetReadWriteFunction(void* WriteFunctionPtr(void *), void* rt_read_task(void*)) 
 // this->SetReadWriteFunction( rt_write_task, rt_read_task) ;
 // this->SetReadWriteFunction( DUMMY_write_task,DUMMY_read_task) ;
}
*/

/*
//controller::controller(void* rt_write_task, void* rt_read_task)
//void controller::SetReadWriteFunction(std::function<void*(void*)> WriteFunctionPtr, std::function<void*(void*)> rt_read_task)
controller::controller(void* rt_write_task(void *) , void* rt_read_task(void *) )
//controller::controller(void* rt_write_task(void *), void* rt_read_task(void *) ) 
{
  this->SetReadWriteFunction( rt_write_task, rt_read_task) ;

}
*/

controller::controller(int writesocket_fd,int readsocket_fd)
{
	
	/*initialize rx and tx can messages structs*/
	for (int i = 0; i < 8; ++i)
	{	
		this->tx_msg.cframe.data[i] = 0;
		this->rx_msg.cframe.data[i] = 0;
	}
	this->tx_msg.cframe.can_dlc = 8; 
	this->rx_msg.cframe.can_dlc = 8; 
	this->write_socket = writesocket_fd;
	this->read_socket = readsocket_fd;
	printf("read socket: %d, write socket: %d\n",read_socket,write_socket );

        RequestMsg.set_socket(writesocket_fd) ;
        
}
#define READ_DEBUG false
bool controller::can_read()
{	
	struct can_frame sframe;	
	int nbytes;
        int N_READ = 1 ;
	//printf("tread 2 socket: %d can_id: %x\n",read_socket,sframe.can_id);	
	nbytes = read(read_socket, &(rx_msg.cframe), N_READ*sizeof(can_frame));
        bit_masking(this->rx_msg);
	
	if (READ_DEBUG) {
		uint32_t ReceivedCanNodeID = this->rx_msg.node_id;
		uint32_t ReceivedCmdID = this->rx_msg.cmd_id;
		sframe = this->rx_msg.cframe;
	    printf("%s : read from  socket: %d can_NODE_id: %x Cmd_id : %x\n", __func__ , read_socket,ReceivedCanNodeID ,ReceivedCmdID );	
		//printf("data read nbytes:%d %d %d %d %d %d %d %d %d. Yay!\n",nbytes,sframe.data[0], sframe.data[1], sframe.data[2], sframe.data[3],sframe.data[4], sframe.data[5], sframe.data[6], sframe.data[7]); 
	}
 /*
 * Controller Area Network Identifier structure
 *
 * bit 0-28     : CAN identifier (11/29 bit)
 * bit 29       : error message frame flag (0 = data frame, 1 = error message)
 * bit 30       : remote transmission request flag (1 = rtr frame)
 * bit 31       : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 typedef __u32 canid_t;
 */
 
	if (nbytes < 0) {
		perror("can raw socket write");
		return 0;
	}

	/* paranoid check ... */
	if (nbytes < N_READ * sizeof(can_frame)) {
		perror("read: incomplete CAN frame\n");
		return 0;
	}
	else if (nbytes > N_READ * sizeof(can_frame)) {
		perror("read failed\n");
		return 0;
	}
	return 1;

}

bool controller::can_write()
{	
	int nbytes;
        int N_WRITE = 1 ;
	nbytes = write(this->write_socket, &(tx_msg.cframe), N_WRITE * sizeof(can_frame));
	if (nbytes < 0) {
		perror("can raw socket write");
		return 0;
	}

	/* paranoid check ... */
	if (nbytes < N_WRITE * sizeof(can_frame)) {
		perror("write: incomplete CAN frame\n");
		return 1;
	}
	else if (nbytes < N_WRITE * sizeof(can_frame)) {
		perror("write: incomplete CAN frame\n");
		return 1;
	}
	return 1;

}
#define DEBUG_ProcessData false 
// Processing Of Data 
void controller::ProcessData() 
{
	uint8_t a,b,c,d,e,f,g,h; //temp storage variale for the CAN data packets
	int i;
	bit_masking(rx_msg);

	a = this->rx_msg.cframe.data[0];	
	b = this->rx_msg.cframe.data[1];
	c = this->rx_msg.cframe.data[2];
	d = this->rx_msg.cframe.data[3];

	e = this->rx_msg.cframe.data[4];
	f = this->rx_msg.cframe.data[5];
	g = this->rx_msg.cframe.data[6];
	h = this->rx_msg.cframe.data[7];



	/*read the received can frame*/
	if (DEBUG_ProcessData) printf(" %s  CANID : %x cmd: %x \n",__func__,  rx_msg.node_id, rx_msg.cmd_id);
	i = rx_msg.node_id;
    switch(this->rx_msg.cmd_id)
    {
        case 1: //hearbeat message
            //printf("controller::msg_handler Revceived HEARTBEAT MSG\n") ;
            this->motors[i].axis_error = (a | b << 8 | c << 16 | d << 24);
            this->motors[i].axis_current_state = (e | f << 8 | g << 16 | h << 24);
            break;
        case 2: //get motor error//
            //printf("controller::msg_handler Received MOTOR ERROR \n") ;
            this->motors[i].motor_error = (a | b << 8 | c << 16 | d << 24);
            break;
        case 4: //get encoder error
            this->motors[i].encoder_error = (e | f << 8 | g << 16 | h << 24);
            break;
        case 9: //get encoder estimates
            uint32_t RTRBit;
            RTRBit = this->rx_msg.cframe.can_id & (1 << 30);
            if (!RTRBit) 
            {
                //printf("%s controller::msg_handler Received ENCODER ESTIMATES \n",__func__) ;
                bytes2Float(&rx_msg.cframe.data[0], &this->motors[i].encoder_pos_estimate); //FLOAT 
                bytes2Float(&rx_msg.cframe.data[4], &this->motors[i].encoder_vel_estimate );  //FLOAT 
                if (DEBUG_ProcessData) printf("%s controller::msg_handler CanID %d ENCODER ESTIMATES %f\n",__func__, i, this->motors[i].encoder_pos_estimate) ;
            }
            break;
        case 10: //get encoder counts
            this->motors[i].encoder_shadow_count = (a | b << 8 | c << 16 | d << 24);  //SIGNED INT 
            this->motors[i].encoder_count_in_cpr =(e | f << 8 | g << 16 | h << 24);  //SIGNED INT 
            break;
        case 20: //get IQ
            bytes2Float(&rx_msg.cframe.data[0], &this->motors[i].iq_setpoint );  //FLOAT 
            bytes2Float(&rx_msg.cframe.data[4], &this->motors[i].iq_measured );  //FLOAT 
            break;
        case 23://vbus voltage
            bytes2Float(&rx_msg.cframe.data[0], &motors[i].vbus_voltage);  //FLOAT 
            break;
    }
}

// End of Processing Data 



void controller::msg_handler()
{
	uint8_t a,b,c,d,e,f,g,h; //temp storage variale for the CAN data packets
	int i;

	if (!can_read()) {
            printf("controller::msg_handler Error in can_read()\n") ;
            exit(-1) ;
        };
        ProcessData();
      /*
	bit_masking(rx_msg);

	a = this->rx_msg.cframe.data[0];	
	b = this->rx_msg.cframe.data[1];
	c = this->rx_msg.cframe.data[2];
	d = this->rx_msg.cframe.data[3];

	e = this->rx_msg.cframe.data[4];
	f = this->rx_msg.cframe.data[5];
	g = this->rx_msg.cframe.data[6];
	h = this->rx_msg.cframe.data[7];



	// read the received can frame 
        
	//printf("controller::msg_handler CANID : %x cmd:%d\n", rx_msg.cframe.can_id,rx_msg.cmd_id);
	i = rx_msg.node_id;
	switch(this->rx_msg.cmd_id)
	{
		case 1: //hearbeat message
	                //printf("controller::msg_handler Revceived HEARTBEAT MSG\n") ;
			this->motors[i].axis_error = (a | b << 8 | c << 16 | d << 24);
			this->motors[i].axis_current_state = (e | f << 8 | g << 16 | h << 24);
			break;
		case 2: //get motor error//
	                //printf("controller::msg_handler Received MOTOR ERROR \n") ;
			this->motors[i].motor_error = (a | b << 8 | c << 16 | d << 24);
			break;
		case 4: //get encoder error
			this->motors[i].encoder_error = (e | f << 8 | g << 16 | h << 24);;
			break;
		case 9: //get encoder estimates
	                //printf("controller::msg_handler Received ENCODER ESTIMATES \n") ;
			bytes2Float(&rx_msg.cframe.data[0], &this->motors[i].encoder_pos_estimate); //FLOAT 
			bytes2Float(&rx_msg.cframe.data[4], &this->motors[i].encoder_vel_estimate );  //FLOAT 
			break;
		case 10: //get encoder counts
			this->motors[i].encoder_shadow_count = (a | b << 8 | c << 16 | d << 24);  //SIGNED INT 
			this->motors[i].encoder_count_in_cpr =(e | f << 8 | g << 16 | h << 24);  //SIGNED INT 
			break;
		case 20: //get IQ
			 bytes2Float(&rx_msg.cframe.data[0], &this->motors[i].iq_setpoint );  //FLOAT 
			 bytes2Float(&rx_msg.cframe.data[4], &this->motors[i].iq_measured );  //FLOAT 
			break;
		case 23://vbus voltage

			 bytes2Float(&rx_msg.cframe.data[0], &motors[i].vbus_voltage);  //FLOAT 
			break;
	}

	// error detection and handling
    */
}	

odrive_motor* controller::get_motor_data(int x)
 {
        int MOTOR_COUNT = 3;
 	if( x < 0 || x > MOTOR_COUNT - 1 )
 	{
 		cout << " Invalid access" << endl;
 		return NULL;
 	}
       /*
        printf( "Motor CanID : %d axis_error : %x \n",x, motors[x].axis_error) ; 
        printf( "Motor CanID : %d axis_current_state: %x \n",x, motors[x].axis_current_state) ; 
        printf( "Motor CanID : %d motor_error : %x \n",x, motors[x].motor_error) ; 
        printf( "Motor CanID : %d encoder_error : %x \n",x, motors[x].encoder_error) ; 
        printf( "Motor CanID : %d encoder_pos_estimate : %f \n",x, motors[x].encoder_pos_estimate) ; 
        printf( "Motor CanID : %d encoder_vel_estimate : %f \n",x, motors[x].encoder_vel_estimate) ; 
        printf( "Motor CanID : %d iq_setpoint : %f \n",x, motors[x].iq_setpoint ) ; 
        printf( "Motor CanID : %d iq_measured : %f \n",x, motors[x].iq_measured ) ; 
        printf( "Motor CanID : %d vbus_voltage : %f \n",x, motors[x].vbus_voltage ) ; 
       */
 	return &this->motors[x];
 }

bool controller::set_axis_to_state( uint32_t canID, int32_t axis_requested_state) 
{
  
  set_axis_requested_state(canID,  axis_requested_state);
  /* if (get_axis_state(canID) != axis_requested_state) 
    return (false);
  */
  return(true) ;
}

/*IMPLEMENTATION OF ALL ODRIVE FUNCTIONS EXCEPT FOR REQUEST MSGS*/

void controller::estop(uint32_t node_id)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = ESTOP;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	can_write();
}
void controller::set_axis_node_id(uint32_t node_id, uint16_t axis_can_node_id)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_AXIS_NODE_ID;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	this->tx_msg.cframe.data[0] = axis_can_node_id & BIT_MASK_0;
	this->tx_msg.cframe.data[1] = (axis_can_node_id & BIT_MASK_1) >>8;
	can_write();
}
void controller::set_axis_requested_state(uint32_t node_id, uint32_t axis_requested_state)
{
	/*
		AXIS_STATE_UNDEFINED = 0
		AXIS_STATE_IDLE = 1
		AXIS_STATE_STARTUP_SEQUENCE = 2
		AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
		AXIS_STATE_MOTOR_CALIBRATION = 4
		AXIS_STATE_SENSORLESS_CONTROL = 5
		AXIS_STATE_ENCODER_INDEX_SEARCH = 6
		AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
		AXIS_STATE_CLOSED_LOOP_CONTROL = 8
		AXIS_STATE_LOCKIN_SPIN = 9
		AXIS_STATE_ENCODER_DIR_FIND = 10
	*/

	tx_msg.node_id = node_id;
	tx_msg.cmd_id = SET_AXIS_REQUESTED_STATE;
	tx_msg.cframe.can_id = (tx_msg.cmd_id | tx_msg.node_id << 5);
	tx_msg.cframe.data[0] = axis_requested_state & BIT_MASK_0;
	tx_msg.cframe.data[1] = ((axis_requested_state  & BIT_MASK_1 )>> 8 );
	tx_msg.cframe.data[2] = ((axis_requested_state  & BIT_MASK_2 )>> 16);
	tx_msg.cframe.data[3] = ((axis_requested_state  & BIT_MASK_3 )>> 24);	
	can_write();
}

//void controller::move_to_pos(uint32_t node_id, int32_t pos)
void controller::move_to_pos(uint32_t node_id, float pos)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = MOVE_TO_POS;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	float2Bytes(pos, this->tx_msg.cframe.data);
        /* Code changed for pos from int32_t to float
	this->tx_msg.cframe.data[0] = pos & BIT_MASK_0;
	this->tx_msg.cframe.data[1] = (pos & BIT_MASK_1) >> 8;
	this->tx_msg.cframe.data[2] = (pos & BIT_MASK_2) >> 16;
	this->tx_msg.cframe.data[3] = (pos & BIT_MASK_3) >> 24;	
        */
	can_write();
}

void controller::set_pos_setpoint(uint32_t node_id, float pos_setpoint, float vel_ff,float current_ff)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_POS_SETPOINT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);

	float2Bytes(pos_setpoint, tx_msg.cframe.data);
	float2Bytes(vel_ff, &(tx_msg.cframe.data[4]) );
	float2Bytes(current_ff, &(tx_msg.cframe.data[6]) );
	can_write();
}

void controller::set_vel_setpoint(uint32_t node_id, float vel_setpoint , float current_ff)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_VEL_SETPOINT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	this->tx_msg.cframe.data[0] = (int32_t(vel_setpoint*100) & BIT_MASK_0);
	this->tx_msg.cframe.data[1] = (int32_t(vel_setpoint*100) & BIT_MASK_1) >> 8;
	this->tx_msg.cframe.data[2] = (int32_t(vel_setpoint*100) & BIT_MASK_2) >> 16;
	this->tx_msg.cframe.data[3] = (int32_t(vel_setpoint*100) & BIT_MASK_3) >> 24;

	this->tx_msg.cframe.data[4] = (int16_t(current_ff*100) & BIT_MASK_0);
	this->tx_msg.cframe.data[5] = (int16_t(current_ff*100) & BIT_MASK_1) >> 8;
	can_write();
}

void controller::set_cur_setpoint(uint32_t node_id, float cur_setpoint)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_CUR_SETPOINT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	this->tx_msg.cframe.data[0] = (int32_t(cur_setpoint*100) & BIT_MASK_0);
	this->tx_msg.cframe.data[1] = (int32_t(cur_setpoint*100) & BIT_MASK_1) >> 8;
	this->tx_msg.cframe.data[2] = (int32_t(cur_setpoint*100) & BIT_MASK_2) >> 16;
	this->tx_msg.cframe.data[3] = (int32_t(cur_setpoint*100) & BIT_MASK_3) >> 24;
	can_write();	
	
}

void controller::set_vel_limit(uint32_t node_id, float vel_limit)  /*FLOAT*/
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_VEL_LIMIT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
		
	float2Bytes(vel_limit, tx_msg.cframe.data);
	//printf("inside class to %x of size %d of socket %d:cframe.data is %x %x %x %x\n", tx_msg.cframe.can_id, tx_msg.cframe.can_dlc, write_socket, tx_msg.cframe.data[0], tx_msg.cframe.data[1], tx_msg.cframe.data[2],tx_msg.cframe.data[3]);
  	can_write();
}
void controller::start_anticogging(uint32_t node_id)
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = START_ANTI_COGGING;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);
	
	can_write();	
}
void controller::set_traj_vel_limit(uint32_t node_id , float traj_vel_limit) /*FLOAT*/
{
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_TRAJ_VEL_LIMIT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);

	float2Bytes(traj_vel_limit, this->tx_msg.cframe.data);
	
	can_write();	
}

void controller::set_traj_accel_limit(uint32_t node_id , float accel_limit, float decel_limit)  /*FLOAT*/
{
	u_int8_t bytes_temp[4];

	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_TRAJ_ACCEL_LIMIT;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);

	float2Bytes(accel_limit, this->tx_msg.cframe.data);
	float2Bytes(decel_limit, &(this->tx_msg.cframe.data[4]));

	can_write();	

}

void controller::set_traj_a_per_css(uint32_t node_id , float traj_a_per_css) /*FLOAT*/
{
	
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_TRAJ_A_PER_CSS;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);

	float2Bytes(traj_a_per_css, this->tx_msg.cframe.data);
			
	can_write();	
	
}

void controller::reboot_odrive(uint32_t node_id)
{
	
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = REBOOT_ODRIVE;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);	
		
	can_write();	
}


void controller::set_vel_pi_gain(uint32_t node_id, float vel_p_gain, float vel_i_gain)  /*FLOAT*/
{
	
	this->tx_msg.node_id = node_id;
	this->tx_msg.cmd_id = SET_VEL_PI_GAIN;
	this->tx_msg.cframe.can_id = (this->tx_msg.cmd_id | this->tx_msg.node_id << 5);

	float2Bytes(vel_p_gain, this->tx_msg.cframe.data);
	float2Bytes(vel_i_gain, &(this->tx_msg.cframe.data[4]));

	can_write();	
}

bool controller::InitialiseAxis(uint32_t CanID)
{
       /* TODO , the following code is not tested yet
          when the limit switch is hit the index search has to reverse the direction and this has to be set
        	**** setting the  AXIS_STATE_ENCODER_DIR_FIND = 10 may possibly help **** 
	int limit_switch_gpio_value = gpio_read(pi, limit_switch_id_[joint_id]);
	ROS_INFO("Value  of Limit Switch =%d", limit_switch_gpio_value);
	if (limit_switch_gpio_value == 1)
	{   
		ROS_ERROR(" %s NOT HANDLING THe CONDITIONS when LimitSwitch is hit before Encoder Search happens ", __func__);
		ROS_ERROR (" TODO , the reverse of direction for index search can be achieved by setting the joint in "0;
				exit(-1);
				ROS_INFO("Limits switch hit, reversing index search joint id:= %s", joint_names_[joint_id].c_str());
				// These code is TODO , not required is absolute encoder is implemented
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
	}
       */
				// Set the AXISSTATE to Index Search and wait for Idle
	printf("Putting Motor Into Idle \n") ;
	set_axis_requested_state(CanID,AXIS_STATE_IDLE);
	WAITFORCHANGEOFSTATE();
        //TODO:MR KEEP BASED ON INDEX FOUND
         FILE* inputFileFlag = fopen("/home/ubuntu/pragati/src/odrive_control/config/joint_homing_flag.txt", "r");
         int joint_homing_flag;
         // Read three values into the joint_homing_values array
         int fscanf_result= fscanf(inputFileFlag, "%d", &joint_homing_flag); 
         printf("READ FLAG VALUE  %d\n", joint_homing_flag);
         printf("FSCANF_RESULT  %d\n", fscanf_result);
         printf("Deciding on INDEX SEARCH \n") ;
         if (fscanf_result== 1 && joint_homing_flag == 0)
        {
        printf("Doing Index Search \n") ;
       	set_axis_requested_state(CanID,AXIS_STATE_ENCODER_INDEX_SEARCH );
	WAITFORENCODEINDEXSEARCH() ;
        }
        fclose(inputFileFlag);
	if (motors[CanID].axis_error ) {
		printf("%s motors[CanID].error : %ld \n", __func__, motors[CanID].axis_error) ;
		return (motors[CanID].axis_error);
	}
	// Run the AXIS to Closed Loop Control
	printf("Putting Motor Into Closed Loop \n") ;
	set_axis_requested_state(CanID,AXIS_STATE_CLOSED_LOOP_CONTROL );
    WAITFORCHANGEOFSTATE();
	if (motors[CanID].axis_error ) {
		printf("%s motors[CanID].error : %ld \n", __func__, motors[CanID].axis_error) ;
	}
	return (motors[CanID].axis_error);
}

/* TODO : code yet to be written
// odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_VEL, reverse_vel); // these 3 values are 
void controller::WriteProperty(uint32_t PropertyName, float PropertyValue ) {
  // TODO writing ANY value 
  printf("Not implmented this yet");
  exit(-1);
}
*/

/* TODO : code yet to be written
//   odrive_serial_interface_[joint_id]->WriteProperty(ODRV_INDEX_SEARCH_REVERSE_VEL, reverse_vel); // these 3 values are  
bool controller::SetAxisState(uint32_t CanID ,  uint32_t RequestedAxisState )
{  // TODO writing ANY value 
  set_axis_requested_state(CanID, RequestedAxisState);
  WAITFORREADCYCLETIME() ;
  return (motors[canId].axis_error() );
}
*/

/* TODO : code yet to be written
//    odrive_serial_interface_[joint_id]->ReadProperty(ODRV_INDEX_FOUND_R, &response);
void controller::IndexFound()
{  // TODO writing ANY value 
  printf("Not implmented this yet");
  exit(-1);
  return (Motors[can_id].
}
*/

#define DEBUG_GET_POS_ESTIMATE   false
float controller::GetPositionEstimate(uint32_t CanID)
{
	// Return the position Estimate which is read by the ProcessData Routine
	 return motors[CanID].encoder_pos_estimate;

  // ***** THE FOLLOWING CODE IS REDUNDANT AND WILL BE NEVER EXECUTED, this code will be done in ProcessData
   // The following is a request to send the encoder estimate for motor with CANID
   RequestMsg.get_encoder_estimate(CanID);
   // sleep(READCYCLETIME) ;// WaitForReadCycleTime() ;
   double StartTime = CurrentTimeInMillisec();
   uint8_t a, b, c, d, e, f, g, h; // temp storage variale for the CAN data packets
   int i;
   // Keep the old positionEstimate 
   float OldPositionEstimate = 	motors[CanID].encoder_pos_estimate;
   // TODO : for recievd msgs which are NOT encoderPosition and Velocity Estimate the
   // TODO   code has to handle it the normal msg_handler mechanism.
   // TODO   We are not handling it now and will be lost

   // TODO : Currently all sent messages are also looped back and this code is processing this.
   // TODO : Need to set a filter in the driver to process only Received messages
   while (true)
   {
		if (!can_read())
		{
			printf("controller::msg_handler Error in can_read()\n");
			sleep(2);
			exit(-1);
		};
		bit_masking(this->rx_msg);
		uint32_t ReceivedCanNodeID = this->rx_msg.node_id;
		uint32_t ReceivedCmdID = this->rx_msg.cmd_id;
		uint32_t RTRBit = this->rx_msg.cframe.can_id & (1 << 30);

		/*
		if (DEBUG_GET_POS_ESTIMATE) {
			printf("controller::msg_handler ReceivedCanNodeID : %d, CMDID %d() %X\n",ReceivedCanNodeID,  ReceivedCmdID,RTRBit ) ;
		}
		*/

		if ((!RTRBit) && (ReceivedCanNodeID == CanID) && (ReceivedCmdID == GET_ENCODER_ESTIMATE))
		{ 
                        // 9 is Encoder Estimate
			// cmd_id value 9: //get encoder estimates
			// printf("controller::msg_handler Received ENCODER ESTIMATES \n") ;
			// GetEncoderEstimates.pos = can_getSignal<float>(msg, 0, 32, true, 1, 0);
			// GetEncoderEstimates.vel = can_getSignal<float>(msg, 32, 32, true, 1, 0);
	
		 	bytes2Float(&rx_msg.cframe.data[0], &motors[CanID].encoder_pos_estimate); //FLOAT
		 	bytes2Float(&rx_msg.cframe.data[4], &motors[CanID].encoder_vel_estimate );  //FLOAT
			if (DEBUG_GET_POS_ESTIMATE)
				printf("controller::msg_handler bytes2Float format encoder_pos_estimate: %f, encoder_vel_estimate : %f\n",motors[CanID].encoder_pos_estimate ,motors[CanID].encoder_vel_estimate ) ;


			// odrive_motor* MotorData = get_motor_data(CanID);
			if (DEBUG_GET_POS_ESTIMATE)
			{
				float Value = (CurrentTimeInMillisec() / 1000) - (StartTime / 1000);
				printf(" Time in seconds  for GetPosEstimate %f \n", Value);
			}
			return motors[CanID].encoder_pos_estimate;
		}
		else
		{
		   if (DEBUG_GET_POS_ESTIMATE) {
			printf("%s ReceivedCanNodeID : %d, CMDID %d %X\n",__func__, ReceivedCanNodeID,  ReceivedCmdID,RTRBit ) ;
		   }
	        // TODO : handle the message which does not have 	ReceivedCmdID == GET_ENCODER_ESTIMATE	
			if (ReceivedCmdID != GET_ENCODER_ESTIMATE) {
                                 if (DEBUG_GET_POS_ESTIMATE)  printf("%s Received unrequested data, calling ProcessData()\n",__func__);
                                 ProcessData();
			}
			if (CheckForTimeOut(CANBUSTIMEOUTVALUE, StartTime))
			{
				printf("TimeOutError waiting for GetPositionEstimate\n");
				printf("TimeOutError Returning the previous Position Estimate\n");
                                // will exit if there is no response from the controller
                                // If such error occurs then there is a fault/bug in the system
				printf(" %s :: Exiting the system \n",__func__);
				sleep(2);
				exit(-1);
				return motors[CanID].encoder_pos_estimate;
			}
		}
   }
}

long controller::GetAxisError(uint32_t canid ) {
	// Axis Errors are returned in heartbeat message.
	// Can be directly read the motors[canid] value
	return(this->motors[canid].axis_error) ;
} 	

request_msg::request_msg()
{
	this->write_msg.cframe.can_dlc = 8;

}

void request_msg::get_motor_error(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_MOTOR_ERROR;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}

void request_msg::get_encoder_error(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_ENCODER_ERROR;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}

void request_msg::get_encoder_estimate(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_ENCODER_ESTIMATE;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}

void request_msg::get_encoder_counts(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_ENCODER_COUNTS;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}

void request_msg::get_iq_values(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_IQ_VALUES;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}
void request_msg::get_vbus_voltage(uint32_t node_id)
{
	this->write_msg.node_id = node_id;
	this->write_msg.cmd_id = GET_VBUS_VOLTAGE;
	this->write_msg.cframe.can_id = (this->write_msg.cmd_id | this->write_msg.node_id << 5 | RTR<<30);
	this->send_msg();
}


int request_msg::send_msg()
{
	int nbytes;

	//printf(" request_msg::send_msg()  Socket FD: %d \n",socket_handler);
	nbytes = write(this->socket_handler, &this->write_msg.cframe, sizeof(struct can_frame));
	//printf("Wrote %d bytes requesting data from ODrive\n", nbytes);

	if (nbytes < 0) {
            perror("can raw socket write");
            return 0;
    }

    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
            perror("write: incomplete CAN frame\n");
            return 0;
    }
    else if (nbytes > sizeof(struct can_frame)) {
            perror("write failed\n");
            return 0;
    }
    return 1;

}

void request_msg::set_socket(int socket_fd)
{
	printf(" request_msg::set_socket(int socket_fd) setting Socket FD: %d \n",socket_fd);
	this->socket_handler = socket_fd;
}
using namespace std;

void controller::CleanUpCanBusInterface()
{
    /* clean up the RT tasks*/
        //write_task.end_periodic_task();
	//read_task.end_periodic_task();
	//tasks[0]->end_periodic_task();
	//tasks[1]->end_periodic_task();
   printf("%s Calling controller:CleanUpCanBusInterface function\n",__func__);
}


