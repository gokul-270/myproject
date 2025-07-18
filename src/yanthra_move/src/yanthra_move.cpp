
// Last Updated : 24JUN2025
//History :
// 06SEP2023 : replaced xyz_to_polar to ConvertXYZToPolarFLUROSCoordinates
// 23AUG2023 - The following changes are getting done(to fix a bug in ros publishing and ensure that yanthra_move deals with 
//             ARM system and odrive_control deals with the individual motors . Currently the functions are distributed across
//  1.All commands from Yanthra_move to the odrive_controller passed values in units of the appropriate joints  (meters or rotations)
//  2. Publishing of joint states values are WRT arm origin and orientation.
//  3. URDF L5 joint5 origin is now at the arm's origin ie yanthra_link.
//  4. Modifiying joint5_homing_position to 0.274 ( read from yanthra_move.yaml)
//  5. Adding a pose called ParkingPose. After initialisation the arm will move to the parkingPose
//  6. During operation Arm Will move to a pose defined in the yaml file
//5Jan2021
// Added ability to read aruco markers (No 23) so that it is good for testing in the lab
// without the need for testing cotton in the lab.
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

//#include <odrive_control/generic_hw_control_loop.h>
#include <odrive_control/generic_hw_control_loop.h>
#include <odrive_control/odrive_hw_interface.h>

#include <yanthra_move/yanthra_move.h>
#include <yanthra_move/joint_move.h>
#include <yanthra_move/yanthra_io.h>
//#include "cotton_detection/capture_cotton_srv.h"
#include <sys/time.h>
#include <time.h>
#include <string>

double start_time, end_time, last_time; 
bool height_scan_enable;
double height_scan_min;
double height_scan_max;
double height_scan_step;
bool Global_vaccum_motor = false ;
bool Trigger_Camera = false ;
bool continous_vaccum =  true ;
bool End_effector_enable;
double joint5_vel_limit;
float l2_homing_sleep_time;
float l2_step_sleep_time;
float l2_idle_sleep_time;
float MIN_SLEEP_TIME_FORMOTOR_MOTION = 0.5 ; // This is the time Given for the Motors to settle down before starting the next task
float CottonCaptureDetectWaitTime = 1; // This is a graacetime for the Camera to Capture&Detect
float FOVThetaMax = 0.3926 ; // 22.5 degrees 
float FOVPhiMax = 0.3926 ;   //22.5  degrees
bool jerk_enabled_phi;
bool jerk_enabled_theta;
float theta_jerk_value;
float phi_jerk_value;
float pre_start_delay;
float EERunTimeDuringL5ForwardMovement = 0.250;
float EERunTimeDuringL5BackwardMovement = 0.500;
float EERunTimeDuringReverseRotation= 0.500;

bool YanthraLabCalibrationTesting;
bool EndEffectorDropConveyor;
int picked = 0;
double joint2_old_pose = 0.001;
int ERROR = 4;
int USER_INPUT = 1;
int INPROCESS = 2;
int SHUTDOWN = 3;
float MIN_SLEEP_TIME_FOR_COTTON_DROP= 2 ;
float MIN_SLEEP_TIME_FOR_COTTON_DROP_FROM_EEF= 0.5 ;
#define MaxFileLength  1024 
char FilePathName[MaxFileLength] ;
std::string PRAGATI_INSTALL_DIR = "/home/ubuntu/Desktop/pragati/" ;
char PRAGATI_INPUT_DIR[MaxFileLength] ;
char PRAGATI_OUTPUT_DIR[MaxFileLength] ;

int vaccum_motor_on_pin = 24;
//int vaccum_motor_pwm_pin = 18;
//int vaccum_motor_highspeed_pin = 25;
int vaccum_on = 1;
int vaccum_off = 0;
int end_effector_on_pin = 21;
int end_effector_direction_pin = 13;
int end_effector_drop_on = 19;
int end_effector_drop_direction = 12;
int end_effector_on = 1;
int end_effector_off = 0;
int CLOCKWISE = 1 ;
int ANTICLOCKWISE = 1 ;
int DROP_EEF = 1 ;
int STOP_EEF = 0 ;
int EndEffectorDirection = CLOCKWISE ;
int end_effector_direction_cw = 1;
int end_effector_direction_acw =0;
int green_led_pin = 4;
int red_led_pin = 15;
int led_on = 0;
int led_off = 1;
int cotton_drop_servo_pin = 12;
int transport_servo_pin = 14;
int solenoid_shutter_pin = 18;//This Pin is Assigned was fan previously
int camera_led_pin = 17;
int shutdown_switch = 2; 
int start_switch = 3;
int start_switch_pulses = 0;
int shutdown_switch_pulses = 0;
int no_of_times_status_update = 0;
double update_start_time;
double current_update_time;
double HARDWARE_TIMEOUT_VALUE; 
//ros::Rate loop_rate(10) ; //#By Mani Radharishnan :changing from 100 to 50
boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface ;

bool PublisherThread =   true ;

// Global Pin definition (*** SEEMS to be double definition ****)
/*
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
    // LED and Polariser working if Camera is enabled 
#if CAMERA_EN == true
    d_out led_control(n, "led_control");

#endif
    // Limit Switch I/P for Joint 5 Initialisation 
#if JOINT5_INIT_EN == true
    d_in joint5_switch_in(n, "joint5_switch_in");
    d_out joint5_init_start(n, "joint5_init_start");
#endif
    // START_SWITCH_IN and OUTPUT_INDICAATOR_LED for User to Start Picking operation 
#if START_SWITCH_EN == true
    d_in start_switch_in(n, "start_switch_in");
    d_out start_switch_out(n, "start_switch_out");
#endif
    a_out problem_indicator_out(n, "problem_indicator_out");  //user indicator
    // SHUTDOWN_SWITCH_IN, if start_stop switch is pressed for more than 5sec, initiate shutdown 
#if SHUTDOWN_SWITCH_EN == true
    d_in shutdown_switch_in(n, "shutdown_switch_in");
#endif
*/
// arm status false means "ARM IS NOT READY FOR USER INPUT" true means "ARM IS READY FOR USER INPUT"
#define ARM_READY true
#define ARM_BUSY false
//TODO ADD ARM_STATUS "ERROR" "READY" "BUSY"
std::string arm_status = "busy";

ros::ServiceClient cotton_client;
/*
   MAIN PROGRAM
   */


void VacuumPump(bool VacuumPumpState)
	{

  if(Global_vaccum_motor ==true){
    if(VacuumPumpState== true){
        //set_servo_pulsewidth(pi,vaccum_motor_pin,1800); 
        //set_servo_pulsewidth(pi,vaccum_motor_pin,vaccum_on); 
        //set_servo_pulsewidth(pi,vaccum_motor_pin,2400); 
        gpio_write(pi,vaccum_motor_on_pin,vaccum_on);
        //gpio_write(pi,vaccum_motor_pwm_pin,vaccum_on);
        //ros::Duration(2).sleep();
        //gpio_write(pi,vaccum_motor_highspeed_pin,vaccum_on);
        //ros::Duration(5).sleep();
        //gpio_write(pi,vaccum_motor_highspeed_pin,vaccum_off);
        //ros::Duration(5).sleep();
    }
    if(VacuumPumpState == false){
        //set_servo_pulsewidth(pi,vaccum_motor_pin,vaccum_off); 
        gpio_write(pi,vaccum_motor_on_pin,vaccum_off);
        //gpio_write(pi,vaccum_motor_pwm_pin,vaccum_off);
    }
    }
}

// Function to create a timestamped log file
std::string createTimestampedLogFile(std::string name) {
    // Generate a timestamped filename
    std::time_t now = std::time(nullptr);
    std::tm local_tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S");
    std::string timestamp = oss.str();

    // Create the log file name with the timestamp
    std::string filename = "/home/ubuntu/tcp_test/log_" + name + timestamp + ".log";
    std::ofstream log_file(filename, std::ios::out | std::ios::app);
    
    if (!log_file.is_open()) {
        ROS_ERROR("Failed to open log file: %s", filename.c_str());
        return ""; // Return an empty string if the file cannot be opened
    }

    return filename; // Return the filename if successful
}


// Function to write a formatted message to the log file
void writeFormattedMessageToLogFile(const std::string& filename, const char* format, ...) {
    std::ofstream log_file(filename, std::ios::out | std::ios::app);
    if (log_file.is_open()) {
        // Prepare the formatted message
        char buffer[256]; // Buffer to hold the formatted message
        va_list args;
        va_start(args, format);
        std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        // Write the formatted message to the log file
        log_file << buffer << std::endl;
        log_file.close(); // Close the log file after writing
    } else {
        ROS_ERROR("Failed to write to log file: %s", filename.c_str());
    }
}

void EndEffector(bool EndEffectorCondition)
{
    if(EndEffectorCondition == true){
        if (EndEffectorDirection == CLOCKWISE) {
           // CLOCKWISE IS FORWARD
           gpio_write(pi,end_effector_on_pin,1) ;
           gpio_write(pi,end_effector_direction_pin,1);  
        }
        else {
           // ANTICLOCKWISE IS REVERSE
           gpio_write(pi,end_effector_on_pin,1) ;
           gpio_write(pi,end_effector_direction_pin,0);  
        }
      /*    {                  // end_effector on
            ROS_ERROR("EndEffector() : Failed to Switch ON the EndEffector PIN") ;
            exit(1) ;
        }*/
    }
    if(EndEffectorCondition  == false){

        gpio_write(pi,end_effector_on_pin,0) && (gpio_write(pi,end_effector_direction_pin,0));  

/*{  // end_effector OFF 
            ROS_ERROR("EndEffector() : Failed to Switch OFF the EndEffector Control Pin") ;  
        }		
*/
    }
}

void SetEndEffectorDirection(bool DIRECTION)
{
    /*
    if(DIRECTION == true){
        gpio_write(pi,end_effector_direction_pin,end_effector_direction_cw) &&  gpio_write(pi,end_effector_on_pin,end_effector_direction_acw) ; // end_effector_clockwise rotation
    }
    if(DIRECTION == false){

        gpio_write(pi,end_effector_direction_pin,end_effector_direction_acw); // end_effector_clockwise rotation
    } */
    
   if ( DIRECTION == CLOCKWISE ) 
       EndEffectorDirection = CLOCKWISE ;
   else 
       EndEffectorDirection = ANTICLOCKWISE ;
}

void EndEffectorDrop(bool CONVEYOR)
{
   

   if ( CONVEYOR == DROP_EEF ) 
	   ROS_WARN("END EFFECTOR DROP BELT RUNNING FORWARD ");
   	   gpio_write(pi,end_effector_drop_on,0);
   	   gpio_write(pi,end_effector_drop_direction,1);
   	   //ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP_FROM_EEF).sleep();
   if(CONVEYOR == STOP_EEF)
   	  ROS_WARN("END EFFECTOR DROP BELT STOPS");
          gpio_write(pi,end_effector_drop_on,0);
   	  gpio_write(pi,end_effector_drop_direction,0);

}

void servo_control(unsigned pwm){

    if(set_servo_pulsewidth(pi, gpio_pin_number, pulsewidth)!=0){

        ROS_ERROR("Failed to Switch PWM on  the PIN"); // end_effector_clockwise rotation
    };
}



void green_led_on(){
    gpio_write(pi,green_led_pin,led_on);
    gpio_write(pi,red_led_pin,led_off);
}
void red_led_on(){
    gpio_write(pi,red_led_pin,led_on);
    gpio_write(pi,green_led_pin,led_off);
}

void blink_led_on(){


    gpio_write(pi,red_led_pin,led_on);
    gpio_write(pi,green_led_pin,led_off);
    ros::Duration(1).sleep();
    gpio_write(pi,red_led_pin,led_on);
    ros::Duration(1).sleep();
    gpio_write(pi,red_led_pin,led_off);
    ros::Duration(1).sleep();
    gpio_write(pi,red_led_pin,led_on);

};


void cotton_drop_shutter(){

    //set_servo_pulsewidth(pi,transport_servo_pin,1700);
    //ros::Duration(2).sleep();
    // Open the Vacuum pump shutter
    set_servo_pulsewidth(pi,cotton_drop_servo_pin,500);
    ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();
    //ros::Duration(5).sleep();
    // Close the Vacuum pump shutter
    set_servo_pulsewidth(pi,cotton_drop_servo_pin,1420);
    //ros::Duration(2).sleep();

    //set_servo_pulsewidth(pi,transport_servo_pin,1500);

}

void cotton_drop_solenoid_shutter(){
	set_mode(pi,solenoid_shutter_pin,PI_OUTPUT);
	gpio_write(pi,solenoid_shutter_pin,1);	
        ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();
	gpio_write(pi,solenoid_shutter_pin,0);
}
// This controls the servo motor which opens and close a shutter for transport unit
void transport_shutter(){

    set_servo_pulsewidth(pi,transport_servo_pin,650);
    ros::Duration(5).sleep();
    set_servo_pulsewidth(pi,transport_servo_pin,1500);


}

int CottonDropTime = 10 ;
int  TransportUnitMovingUpTime = 805000 ;
int TransportUnitMovingDownTime = 705000 ;
int TransportUnitSlideOpenPosition = 500 ;
int TransportUnitPauseTime = 2 ;

int TransportUnitActivate() 
{

    // Move the Transport unit down until it hits the limit switch

    //while (1) 
    {
        int pinvalue12 = gpio_read(pi,12);
        printf("value of pin 12 %d OUTPUT) \n", pinvalue12);
        printf("checking for limitswitch\n");
        while ( pinvalue12 == 1)
        {
            // Set the direction to go down and run until it hits the limit switch
            gpio_write(pi,23,1);
            gpio_write(pi,4,0);
            sleep(0.25);
            pinvalue12 = gpio_read(pi,12);
        }

        if ( pinvalue12 == 0)

        {
            // Move the unit a little above , run for 0.25 seconds
            printf("Setting Pin 23 and Pin 4 to 1 1 \n");

            gpio_write(pi,23,1);
            gpio_write(pi,4,1);
            usleep(30000);

            //Drop the cotton
            gpio_write(pi,23,1);
            gpio_write(pi,4,1);
            usleep(110000);

            set_servo_pulsewidth(pi, 14, TransportUnitSlideOpenPosition);  
            sleep(CottonDropTime);

            printf("cotton is dropping \n");
            printf("Setting Pin 23 and Pin 4 to 1 1 \n");
            // Move the TransportUnit Up
            gpio_write(pi,23,1);
            gpio_write(pi,4,1);
            usleep(TransportUnitMovingUpTime);

            // Move The transport unit down
            printf("Setting Pin 23 and Pin 4 to 1  0 \n");
            gpio_write(pi,23,1);
            gpio_write(pi,4,0);
            usleep(TransportUnitMovingDownTime);

            // Stop the Transport unit
            printf("Setting Pin 23 and Pin 4 to 0 0 \n");
            gpio_write(pi,23,0);
            gpio_write(pi,4,0);
            sleep(TransportUnitPauseTime);

            set_servo_pulsewidth(pi, 14, 1200); //waiting for collection
            printf("waiting for collection \n");
            sleep(10);
        }
    }; return(0) ; // returningTrue 
}


void camera_led(bool DIRECTION){


    if(DIRECTION == true){
        gpio_write(pi,camera_led_pin,led_on);
    }
    if(DIRECTION == false){
        gpio_write(pi,camera_led_pin,led_off);
    }
}


bool start_switch_status(){

    if(gpio_read(pi,start_switch) == 0){

        start_switch_pulses++;
        if(start_switch_pulses > 5){

            return true;		
        }
        return false;		

    }
    else{
        start_switch_pulses = 0;
        return false;		
    }
}


bool shutdown_switch_status(){

    if(gpio_read(pi,shutdown_switch) == 0){

        shutdown_switch_pulses++;
        if(shutdown_switch_pulses > 10){

            return true;		
        }
        return false;		

    }
    else{
        shutdown_switch_pulses = 0;
        return false;		
    }
}

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




//bool arm_status_function(yanthra_move::arm_status::Request &req,yanthra_move::arm_status::Response &res){
bool arm_status_function(yanthra_move::arm_status::Request &req,yanthra_move::arm_status::Response &res){


    //if(req.capture == true)
    //res.data  = arm_status;
    //if(req.capture == false)
    //res.data  = arm_status;
    if(arm_status == "busy"){

        no_of_times_status_update++;
        if(no_of_times_status_update == 1){

            update_start_time = currentTimeMillis();
        }
        if(no_of_times_status_update > 1 ){
            ROS_INFO("busy status updated more than one time");
            current_update_time = currentTimeMillis();
            ROS_INFO("time difference in millisecond = %f",(current_update_time - update_start_time));
            if ((current_update_time - update_start_time) > HARDWARE_TIMEOUT_VALUE ){  // 50s  

                ROS_ERROR("HARDWARE TIMEOUT ERROR");
                arm_status = "error";
            }
        }
    }
    if(arm_status != "busy"){

        no_of_times_status_update = 0;

    }
    res.status = arm_status;
    //ROS_WARN("%s",arm_status.c_str());
    return true;
}

void get_cotton_coordinates(std::vector<geometry_msgs::Point> * positions)
{
    std::string path;
#if CALIBRATIONMODE == true
    path = CALIBRATION_MODE_DATA_DIR;
#elif TEST140MODE == true
    path = TEST140_MODE_DATA_DIR;
#else
    if (ARM_CALIBRATION == true){
        path = CALIBRATION_MODE_DATA_DIR;
    }
    else{
        path = YANTHRA_DATA_OUTPUT_DIR;

    }
#endif

    std::string cotton_coordinates_file = path + "/cotton_details.txt";
    std::string CommandLineString ;
    std::string SpaceChars = "  " ;
    std::string CommandEcho = "echo ";
    std::string CommandCopy = "cp ";
    std::string CommandSort = "/home/ubuntu/pragati/scripts/SortCottonCoordinates.py ";
    std::string SortOutputFilePath = "/home/ubuntu/pragati/outputs/CottonCoordinatesSorted.txt" ;
    std::string SortInputFilePath = "/home/ubuntu/pragati/outputs/CottonCoordinatesUnsorted.txt" ;
    //std::string InputFilePath = "/home/ubuntu/pragati/outputs/cotton_details.txt";
    std::string InputFilePath = path + "/cotton_details.txt";
    //ROS_WARN (" Cotton_details.txt File Path : " + InputFilePath );
    CommandLineString = " Cotton_details.txt File Path : " + InputFilePath;
    //ROS_WARN(CommandLineString.c_str());
    
    // echo the command
    CommandLineString =  CommandEcho +  SpaceChars + CommandCopy + SpaceChars + SortInputFilePath  + SpaceChars  + SortOutputFilePath ;
    system (CommandLineString.c_str());

    // Copy the input file to CottonCoordinatesUnsorted.txt
    CommandLineString =  CommandCopy + InputFilePath  + SpaceChars  + SortInputFilePath ;
    system (CommandLineString.c_str());
    CommandLineString = " Copying command ..: " + CommandLineString ;
    //ROS_WARN(CommandLineString.c_str());

    // Run the sort command 
    CommandLineString =  CommandSort + "-i" + SortInputFilePath  + SpaceChars  + "-o" +SortOutputFilePath ;
    system (CommandLineString.c_str());
    CommandLineString = "Sorting command ..: " + CommandLineString ;
    //ROS_WARN(CommandLineString.c_str());

    cotton_coordinates_file = SortOutputFilePath ;
    //ROS_INFO("yanthra_move :: get_cotton_coordinates");
    CommandLineString = "Reading Inputs From File :  " + SortOutputFilePath ;
    //ROS_WARN(CommandLineString);
    //ROS_WARN(  SortOutputFilePath.c_str()) ;
    std::ifstream cotton_fs(SortOutputFilePath);

    // check for cotton_fs  and exit program if not open
    if (!cotton_fs.is_open()) {
        ROS_ERROR("Error: not able to open the file ");
        ROS_ERROR("Exiting yanthra_move ");
        //exit(1) ;
    }

    float tmp, point_3d_x, point_3d_y, point_3d_z;
    geometry_msgs::Point pos;

    positions->clear();
    while(cotton_fs >> tmp >> tmp >> point_3d_x >> point_3d_y >> point_3d_z)
    {
        ROS_INFO( "get_cotton_coordinates X: %f  Y: %f  Z: %f " ,point_3d_x , point_3d_y, point_3d_z) ;
        std::cout << "get_cotton_coordinates " <<point_3d_x << " " << point_3d_y << " " << point_3d_z << std::endl;
        pos.x = point_3d_x;
        pos.y = point_3d_y;
        pos.z = point_3d_z;
        positions->push_back(pos);
    }   
    /*
#if CAMERA_EN == true
    // Pragati_Pending: We need to see if this new line is really required.
    std::string echo_newline = "echo > " + cotton_coordinates_file;
    system(echo_newline.c_str());
#endif
*/
    cotton_fs.close();

}

#include <errno.h>
#include <signal.h>
int ChildProcessId ;
time_t curtime;
//COTTONDETECTPROGRAM = "/home/ubuntu/pragati/launch_files/CottonDetect.py"

bool CameraReady = false;
int NoOfCapturedImages = 0;
int value = 0 ;
void signal_handlerSIGUSR2(int signo)
{
    int pid ;

    //pid = getpid();

    if(signo == SIGUSR2)
    {
        //printf("In signalHandler with SIGUSR1\n");
        //if(pid == parent)
        //    kill(pid_1,SIGUSR1);
        //time(&curtime);
        //kill(pid_2,SIGUSR2);
        //printf("YanthraMoveCommunicationInterface SIGUSR2 handler Received signal at %s \n", ctime(&curtime));
        CameraReady = true ;
    }
    else {
         perror("signal_handlerSIGUSR2 received UNKNOW signal\n");                                                  
    }
}

int SpawnCottonDetectProcess()
{

  //// Code for Blocking and Unblocking Signals ****/
  sigset_t set,set2;
  struct sigaction sigs;
  // Install Signal Handler for SIGUSR2 and SIGALRM
    signal(SIGUSR2,signal_handlerSIGUSR2);
    //signal(SIGALRM,signal_handlerSIGUSR2);

  //sigs.sa_handler = signal_handlerSIGUSR2 ;
  //sigemptyset(&sigs.sa_mask);
  //sigs.sa_flags=SA_ONESHOT;
  // sigaction(SIGUSR2, &sigs,0);
  sigemptyset(&set);
  sigemptyset(&set2);
  //sigaddset(&set,SIGUSR2);
  if (sigaddset (&set, SIGALRM) == -1) {
            perror("SpawnCottonDetectProcess->main() : Sigaddset error SIGALRM");                                                  
            pthread_exit((void *)1);                                                    
  }
  if(sigaddset(&set, SIGUSR2) == -1) {                                           
            perror("SpawnCottonDetectProcessYanthraMove->main() : Sigaddset error SIGUSR2");                                                  
            pthread_exit((void *)1);                                                    
  } 
 
  ROS_INFO("SpawnCottonDetectProcess->main() ***BLOCKING** SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  if(sigprocmask(SIG_BLOCK, &set, NULL)==0){
    printf("SpawnCottonDetectProcess->main() ***BLOCKED** SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  }
  sigpending(&set2);
  if (sigismember(&set2,SIGUSR2)==1)
  {
    ROS_INFO("SpawnCottonDetectProcessThe SIGUSR2 signal is blocked\n");  //it should print this
  }
  if (sigismember(&set2,SIGALRM)==1)
  {
    ROS_INFO("SpawnCottonDetectProcessThe SIGALRM signal is blocked\n");  //it should print this
  }

  //printf("YanthraMoveCommunicationInterface Going to Wait for 2 seconds at   %s\n",ctime(&curtime));
  //sleep(2);
  //printf("YanthraMove -> main() : Sending SIGUSR2 to itself\n");
  // kill(getpid(),SIGUSR2); //the signal shouldn't work

  ROS_INFO("YanthraMoveCommunicationInterface Going to Wait for 2 seconds AGAIN at   %s\n",ctime(&curtime));
  sleep(2);


    int pid_1 ;
    int parentPID = getpid();

    /*
    sigset_t base_mask;
    sigset_t  waiting_mask;
    sigemptyset (&base_mask);
    sigaddset (&base_mask, SIGUSR2);
    */
    //sigaddset (&base_mask, SIGTSTP);
    //
    ROS_INFO("SpawnCottonDetectProcess(): Parent ProcessID : %d \n",parentPID) ;
    if ( (ChildProcessId = fork() ) < -1) {
        ROS_INFO("SpawnCottonDetectProcess ChildProcess ID %d, childProcess FAILED \n", ChildProcessId) ;
        exit(0);
    }
    else if (ChildProcessId == 0) {
        // Execution is in the child process CottonDetectCommunicationInterface
        // We need to Block all Interrupts before the child process unblock the interrupts
        ROS_INFO(" SpawnCottonDetectProcess Execution in ChildProcess proceesID : %d \n", getpid()) ;
        //ROS_INFO("SpawnCottonDetectProcess Spawning ./CottonDetect.py\n")  ;
        //execl ( "./CottonDetectCommunicationInterface", (char *) NULL);
        //execl ( "./OakDCommunicationInterface.py", (char *) NULL);
        execl ( "/home/ubuntu/pragati/src/OakDTools/CottonDetect.py", "CottonDetect", (char *) NULL);
        ROS_INFO("SpawnCottonDetectProcess  ERROR Could Not Spawn /home/ubuntu/pragati/src/OakDTools/CottonDetect.py\n");
        perror("Exec failed");
        exit(EXIT_FAILURE);
        exit(-1);
    }

/*
    else
    {

        // Block user interrupts while doing other processing. 
        // UNBlock all user interrupts now .
        printf(" YanthraMoveCommunicationInterface MASKING SIGUSR2  %s\n",ctime(&curtime));
        sigprocmask (SIG_SETMASK, &base_mask, NULL);

        printf("YanthraMoveCommunicationInterface  ChildProcess ID : %d \n", ChildProcessId) ;
        time(&curtime) ;
        printf(" YanthraMoveCommunicationInterface  Checking  Camera Status at   %s\n",ctime(&curtime));
        if (!CameraReady) printf("YanthraMoveCommunicationInterface  CAMERA NOT READY at %s\n",ctime(&curtime)) ;
        else {
            printf("YanthraMoveCommunicationInterface : ERROR, CameraReady Already True\n");
            exit(-1);
        }
    }
*/
  ROS_INFO("YanthraMoveCommunicationInterface->main() UNBLOCKING SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  if(sigprocmask(SIG_UNBLOCK, &set, NULL)==0){
    ROS_INFO("YanthraMoveCommunicationInterface->main() UNBLOCKED SIGUSR2, SIGALRM at  %s\n",ctime(&curtime));
  }
  ROS_INFO("YanthraMoveCommunicationInterface Going to Wait for 2 seconds at   %s\n",ctime(&curtime));
 
  ROS_INFO("YanthraMoveCommunication - main() CameraReady %s \n", CameraReady ? "TRUE" : "FALSE"); 
  return(0) ;

}


// This is the function called on receipt of signal(cntl+c)
void sig_handler (int sig)
{
    printf("\n YanthraCommunicationInterface sig_handler :: This is signal handler for signal %d\n", sig);
    if( sig == SIGUSR2) {
        CameraReady = true;
        printf("YanthraCommunicationInterface : sig_handler Processing SIGUSR2\n") ;
    }
}

void catch_alarm( int Alarmsignal)
{
    signal (Alarmsignal, catch_alarm);
}

// NOT TESTED YET
int WaitForCameraReady()
{
    /*
    struct sigaction act;
    act.sa_handler = sig_handler;
    sigemptyset(&act.sa_mask);
    sigaddset(&act.sa_mask, SIGUSR2);
    act.sa_flags = 0;

    sigset_t sig_term_kill_mask;
    sigaddset(&sig_term_kill_mask, SIGUSR2);
    //sigaddset(&sig_term_kill_mask, SIGKILL);

    //sigaction(SIGINT, &act, NULL);
    */

    ////// Critical section /////////
    value++;

    ROS_INFO("\n YanthraMove->WaitForCameraReady : Value is %d\n", value);
    int SigWaitReturnValue ;
    if (CameraReady == false) {
        ROS_INFO("YanthraMove->WaitForCameraReady Waiting for signals SIGUSR2 \n");
        /* sigsuspend(&sig_term_kill_mask); */
        sigset_t   set;
        sigemptyset(&set); 
        if (sigaddset (&set, SIGALRM) == -1) {
            perror("YanthraMove YanthraMove->WaitForCameraReady() : Sigaddset error SIGALRM");                                                  
            pthread_exit((void *)1);                                                    
        }
        if(sigaddset(&set, SIGUSR2) == -1) {                                           
            perror("YanthraMove YanthraMove->WaitForCameraReady() : Sigaddset error SIGUSR2");                                                  
            pthread_exit((void *)1);                                                    
        } 
        //alarm(1);
        //Wait for Signal SIGUSR2 or timeout
        if(sigwait(&set,&SigWaitReturnValue )) {                                                 
            perror("YanthraMove YanthraMove->WaitForCameraReady(): Sigwait error");                                                    
            //pthread_exit((void *)2);                                                    
	    } 
        switch (SigWaitReturnValue) {
            case SIGUSR1:
                ROS_INFO("YanthraMove->WaitForCameraReady Recieved SIGUSR1 \n");
                break;
            case SIGUSR2:
                ROS_INFO("YanthraMove->WaitForCameraReady Recieved SIGUSR2 \n");
                // driver exit
                CameraReady = true;
                break;
            case SIGALRM:
                ROS_INFO("YanthraMove->WaitForCameraReady Recieved SIGALRM\n");
                break;
            case SIGTTOU:
                ROS_INFO("YanthraMove->WaitForCameraReady Recieved SIGTTOU\n");
                break;
            default:
                ROS_INFO("YanthraMove->WaitForCameraReady Recieved Default\n");
                break;
        }                                                                   
        printf("\n YanthraMove->WaitForCameraReady Return from suspended successful\n");
    }
    return 0;
}
// This waits for the asynchronous signal handler  handler_SIGUSR2 to update CameraReady when SIGUSR2 is recieved
// There is no call to signal wait etc

void WaitForCameraReadySignal()
{
    int iteration = 0 ;
    //while ( (!CameraReady) && (iteration <= 100) ) {
    while ( (!CameraReady) ) {
        //iteration++ ;
        //TODO use the ROS loop_rate here instead of Hardcoding
        usleep( 100000); // 1/10 seconds
    }
    if (iteration > 100 ) {
        perror("signal_andlerSIGUSR2 TimeOUT\n"); 
        perror("check for ERROR IN CAMERA \n");
        exit(-1) ;
    }
    return ;    
}


// This functions wait for the Signal SIGUSR2 from DetectCotton an asynchronous
// process. Once the signal is received the CameraReady signal is set
// and the action can proceed further to process the data 
void  DeprecatedTriggerCaptureAndDetectCotton()
{
    WaitForCameraReady();
    ROS_INFO(" TriggerCaptureAndDetectCotton:: Pausing for SIGUSR2 signal from CottonDetectProcess\n") ;
    if (CameraReady)
    {
        CameraReady = false;
        ROS_INFO("TriggerCaptureAndDetectCotton :: Sleeping for 1 second \n");
        sleep(1);
        kill(ChildProcessId, SIGUSR1);
        time(&curtime);
        ROS_INFO("TriggerCaptureAndDetectCotton:: Sent Signal to DetectCotton at  %s\n", ctime(&curtime));
    }
    else {
        ROS_INFO("TriggerCaptureAndDetectCotton :: ERROR in WaitingForCameraReady, CameraMay have DIED \n");
        exit(-1);
    }
    
    // WaitFor Camera to Complete, will be True if CameraReady Variable is True 
    WaitForCameraReady();
    // Assume that the Camera has finished processing the images and has written
    // the values in cotton_details.txt file
    //TODO : Handling of Failure conditions from Camera to be handled
}

// and the action can proceed further to process the data 
#define DEBUGCAPTURE false 
void  TriggerCaptureAndDetectCotton()
{
    
    WaitForCameraReadySignal();
    ROS_INFO(" TriggerCaptureAndDetectCotton:: Pausing for SIGUSR2 signal from CottonDetectProcess\n") ;
    CameraReady = false;
    if (DEBUGCAPTURE) ROS_INFO("TriggerCaptureAndDetectCotton :: Sending SIGUSR1 to Camera");
    kill(ChildProcessId, SIGUSR1);
    // WaitFor Camera to Complete, will be True if CameraReady Variable is True 
    WaitForCameraReadySignal();
}

///  Write a Generic StartSwitch function 
/*
bool CheckForShutDownSwitchAndShutDownSystem()
{
    while((start_switch_in.state() == false))// && (start_switch_status() == false))
    {	

        ros::spinOnce();
        ros::Duration(0.1).sleep();
        if((shutdown_switch_in.state() == true || shutdown_switch_status() == true )){
            ROS_ERROR("SHUTDOWN Switch ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM ");
            blink_led_on();
            ros::Duration(2).sleep();
            if (save_logs == true){
                ROS_WARN("copying all the logs");
                system("/home/ubuntu/pragati/launch_files/copy.sh");
            }

            // TODO odrive_hw_controller.ShutDown();

            //problem_indicator_out.command(SHUTDOWN);
            ROS_ERROR("SHUTDOWN SWITCH ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM");
            //system("rosnode kill -a");
            //system("sudo shutdown -P now");
            red_led_on();
            return(false) ;
        }
    }
   return(true ) ;
}
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
  
//boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface ;
  
#define DEBUGJOINTSTATEPUBLISHER false
// The function to be executed by all threads
void *ODriveJointStatePublisher(void *vargp)
{
    // Store the value argument passed to this thread
    //int *myid = (int *)vargp;

    ros::Rate PublishingRate(10) ;
    
    if( DEBUGJOINTSTATEPUBLISHER) ROS_ERROR(" \n %s Starting joint state publisher through the read function\n ",__func__);
    if( DEBUGJOINTSTATEPUBLISHER) ROS_ERROR(" \n %s Starting joint state publisher through the read function\n ",__func__);
    if( DEBUGJOINTSTATEPUBLISHER) ROS_ERROR(" \n %s Starting joint state publisher through the read function\n ",__func__);
    if( DEBUGJOINTSTATEPUBLISHER) ROS_ERROR(" \n %s Starting joint state publisher through the read function\n ",__func__);
    if( DEBUGJOINTSTATEPUBLISHER) ROS_ERROR(" \n %s Starting joint state publisher through the read function\n ",__func__);
    //while(ros::ok())
    while( PublisherThread ) 
    {
        if( DEBUGJOINTSTATEPUBLISHER) ROS_INFO("%s >> In ODriveJointStatePublisher Loop", __func__);
        ros::Duration elapsed_time(0) ;
        odrive_hw_interface->read(elapsed_time );
        odrive_hw_interface->PublishJointStateValues() ;//ODriveJointStatePublisher();
        odrive_hw_interface->write(elapsed_time );
        // Print the argument, static and global variables
        //ROS_INFO("In ODriveJointStatePublisher loop with threadID : %d", *myid) ;
        PublishingRate.sleep() ;
    }
    return (0) ;
}
   

//14-MAR-2023 : Manohar Sambandam , calling this function to workaround the DEADLOCK  THREAD unidentified bug for OdriveControl 
 //              Read-write-loop . TODO need to rewind this change after finding the bug related to DEADLOCK thread
 //Dummy no is passed since it is not used 
void  CallOdriveHWLoop()
{
    ROS_INFO("In CallOdriveHWLoop") ;
    ros::Duration elapsed_time(0) ;
    odrive_hw_interface->read(elapsed_time );
    odrive_hw_interface->PublishJointStateValues() ;
    //odrive_hw_interface->write(elapsed_time );

}

void PrintVersionAndDate( char *ProgramName)
{
#define COMPILE_DATE __DATE__ " " __TIME__
    ROS_WARN(" The  %s Version :  %s  built on : %s\n", ProgramName, VERSION, COMPILE_DATE);
}

int main(int argc, char** argv)
{
    ROS_INFO("Printing Version and Compile Date");
    PrintVersionAndDate(( char*) "yanthra_move " );
    ros::init(argc, argv, "yanthra_move");
    ros::NodeHandle n;
    ros::NodeHandle YanthraNode ;

    std::string log_filename = createTimestampedLogFile("xyz");
                    if (log_filename.empty()) {
                        return 1; // Exit if the log file cannot be created
                         }
    std::string log_movement = createTimestampedLogFile("arm");
                    if (log_movement.empty()) {
                        return 1; // Exit if the log file cannot be created
                         }


//#include <odrive_control/generic_hw_control_loop.h>
//#include <odrive_control/odrive_hw_interface.h>

// Moved this definition to global scope
    //boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
  //boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
  //boost::shared_ptr<odrive_control::ODriveHWInterface> LocalOdriveHwInterface 
  //      (new odrive_control::ODriveHWInterface(n) );

    //Updating the arm status , false for NOT READY, true for READY
    //arm_status = "busy";
    arm_status = "uninit"; //TODO:ManiRadhakrishnan:Publish it as UNINIT to avoid ARM_CLIENT STATUS FAILURE
    ROS_INFO("Publishing Arm Status to ARMCLIENT");
    arm_status_srv_ = n.advertiseService("/yanthra_move/current_arm_status", arm_status_function);
    PRAGATI_INSTALL_DIR = "/home/ubuntu/pragati/" ;
    if (!n.getParam("PRAGATI_INSTALL_DIR", PRAGATI_INSTALL_DIR)) {
        ROS_INFO(" Pragati is Running from the default Location %s", PRAGATI_INSTALL_DIR.c_str());
        //PRAGATI_INSTALL_DIR = "/home/grobomac/Desktop/pragati/";
        PRAGATI_INSTALL_DIR = "/home/ubuntu/pragati/" ;
    }

    sprintf(PRAGATI_INPUT_DIR,"%s/inputs/", PRAGATI_INSTALL_DIR.c_str());
    sprintf(PRAGATI_OUTPUT_DIR,"%s/outputs/", PRAGATI_INSTALL_DIR.c_str());
    std::cout << "COUT : PRAGATI_INSTALL_DIR : " << PRAGATI_INSTALL_DIR << "\n";
    std::cout << "COUT : " << PRAGATI_INPUT_DIR << "\n" ;
    std::cout << "COUT : " << PRAGATI_OUTPUT_DIR << "\n" ;
    ROS_INFO("PRAGATI_INSTALL_DIR: %s", PRAGATI_INSTALL_DIR.c_str());
    ROS_INFO("PRAGATI_OUTPUT_DIR: %s", PRAGATI_OUTPUT_DIR);
    ROS_INFO("PRAGATI_INPUT_DIR: %s", PRAGATI_INPUT_DIR);
    ROS_ERROR("ARUCO_FINDER_PROGRAM: %s", ARUCO_FINDER_PROGRAM); //edited for testing purpose on dec_16
    ROS_ERROR("TESTING THE LINE.........");  //edited for testing purpose on dec_18
 /* Wait for node to start */
    ros::start() ;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(10) ; //#By Mani Radharishnan :changing from 100 to 50
    XmlRpc::XmlRpcValue joint_pose_values;
    //moveit::planning_interface::MoveGroup group("Arm");
    n.param<bool>("YanthraLabCalibrationTesting", YanthraLabCalibrationTesting, false);

    pi = pigpio_start(NULL,NULL);
    if (pi < 0 ) {
        ROS_ERROR("pigpio_start() function did not successful") ;
        exit(1) ;
    }
    // Code section to initiate odrive_hw_interface and /joint_state publisher
    ros::Duration elapsed_time(0) ;
    //boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
    //     (new odrive_control::ODriveHwInterface(n));
    boost::shared_ptr<odrive_control::ODriveHWInterface> LocalOdriveHwInterface 
        (new odrive_control::ODriveHWInterface(n) );
    odrive_hw_interface = LocalOdriveHwInterface ; 
    odrive_hw_interface->init();
    //odrive_hw_interface->read(elapsed_time );
    //odrive_hw_interface->write(elapsed_time );
    //CallOdriveHWLoop();
    // Changed 14MAR2023 : removed the thread to eliminate multi-thread process
    //     the threads are dying for some unknown reasons. We are making a direct call to
    //     CallOdriveHWLoop()
           pthread_t ThreadId;
           PublisherThread = true ;
           pthread_create(&ThreadId, NULL, ODriveJointStatePublisher, (void *) &odrive_hw_interface);
    ROS_INFO("%s Past the creation of OdriveHWInterface\n", __func__);
    sleep(10) ;

#if    TEST140MODE == true
    ROS_WARN(" *** Running TEST140MODE To test Yanthra Robotic Arm **** \n") ;
#endif
    if ( YanthraLabCalibrationTesting ) {
	 ROS_INFO("RUNNING POSITION ACCURACY CHECK USING ARUCO PATTERN \n");
    }
    else {
//  Start the camera process
#if CAMERA_EN == true
    SpawnCottonDetectProcess() ;
#endif
    }

    // configure all pins for the arm
    set_mode(pi,start_switch,PI_INPUT);
    set_mode(pi,shutdown_switch,PI_INPUT);
    set_mode(pi,vaccum_motor_on_pin,PI_OUTPUT);
    set_mode(pi,cotton_drop_servo_pin,PI_OUTPUT);
    set_mode(pi,camera_led_pin,PI_OUTPUT);
    set_mode(pi,end_effector_on_pin,PI_OUTPUT);
    set_mode(pi,end_effector_direction_pin,PI_OUTPUT);
    set_mode(pi,end_effector_drop_on,PI_OUTPUT);
    set_mode(pi,end_effector_drop_direction,PI_OUTPUT);
    set_mode(pi,green_led_pin,PI_OUTPUT);
    set_mode(pi,red_led_pin,PI_OUTPUT);

    set_pull_up_down(pi,start_switch,PI_PUD_UP);
    set_pull_up_down(pi,shutdown_switch,PI_PUD_UP);
    set_pull_up_down(pi,camera_led_pin,PI_PUD_UP);
    set_pull_up_down(pi,green_led_pin,PI_PUD_UP);
    set_pull_up_down(pi,red_led_pin,PI_PUD_UP);

    // Pins used for Transport Unit	
    set_mode(pi,12,PI_INPUT); 
    set_mode(pi,23,PI_OUTPUT); //Enable
    set_mode(pi,4,PI_OUTPUT); // Direction for cytron
    set_mode(pi,14,PI_OUTPUT); //Servomotor pin
    set_pull_up_down(pi,12,PI_PUD_UP);


    // turn off vaccum and camera if it is on
    VacuumPump(false);
    camera_led(false);
    //turn on red_led 
    red_led_on();

    /* Reading Configuration */
    n.param<bool>("continous_operation", continous_operation, true);
    n.param<bool>("save_logs", save_logs, true);
    n.param<bool>("YanthraLabCalibrationTesting", YanthraLabCalibrationTesting, false);
    n.param<bool>("EndEffectorDropConveyor", EndEffectorDropConveyor, false);
    n.param<float>("delays/picking", picking_delay, 0.5);
    n.param<float>("delays/EERunTimeDuringL5ForwardMovement",EERunTimeDuringL5ForwardMovement, 0.250);
    n.param<float>("delays/EERunTimeDuringL5BackwardMovement",EERunTimeDuringL5BackwardMovement, 0.500);
    n.param<float>("delays/EERunTimeDuringReverseRotation",EERunTimeDuringReverseRotation, 0.500);
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
    //ROS_INFO("multiple_zero_poses value read form config file := %d",joint4_multiple_zero_pose);

    n.param<double>("/joint4_init/park_position", joint4_parking_pose, 0.001);
    n.param<double>("joint4_init/homing_position", joint4_homing_position,0.001 );
    n.param<float>("joint4_init/theta_jerk_value",theta_jerk_value,0.0);

    n.param<float>("joint5_init/phi_jerk_value",phi_jerk_value,0.0);
    n.param<float>("l2_step_sleep_time",l2_step_sleep_time,2.5);
    n.param<float>("l2_homing_sleep_time",l2_homing_sleep_time,5.0);
    n.param<float>("l2_idle_sleep_time",l2_idle_sleep_time,2.0);
    n.param<float>("min_sleep_time_formotor_motion",MIN_SLEEP_TIME_FORMOTOR_MOTION,2.0);

    n.param<bool>("arm_calibration",ARM_CALIBRATION,true);
    n.getParam("joint_poses",joint_pose_values);

    n.param<bool>("Continous_vaccum",continous_vaccum, true);
    // reading the timeout error value

    n.param<double>("hardware_timeout", HARDWARE_TIMEOUT_VALUE , 90000);
    ROS_INFO("hardware timeout =  %f",HARDWARE_TIMEOUT_VALUE );


    //ROS_INFO("%f",joint_pose_values[0][1]);
    int size_of_pose =joint_pose_values.size();
    //ROS_INFO("%d",size_of_pose);
    red_led_on();

    //pose values read from yaml file

    for(int i=0; i<size_of_pose; i++ )
    {

        //ROS_INFO("link2 poses %f:=",joint_pose_values[i][0]);
        double joint2_pose =  joint_pose_values[i][0];
        double joint3_pose  = joint_pose_values[i][1];
        double joint4_pose  = joint_pose_values[i][2];
        double joint5_pose  = joint_pose_values[i][3];
        //ROS_INFO("link2 poses %f:=",joint2_pose);
        //ROS_INFO("link2 poses %f:=",joint3_pose);
        //ROS_INFO("link2 poses %f:=",joint4_pose);

    }

    //d_out vaccum_motor(n,"vaccum_motor"); // not used


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
    n.param<bool>("trigger_camera",Trigger_Camera,true);
    n.param<bool>("end_effector_enable",End_effector_enable,true);
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
    /* Changed this routine to add the Odrive joint_id a hardcoded value 
    joint_move joint_move_3(n, "joint3_position_controller/");
    joint_move joint_move_4(n, "joint4_position_controller/");
    joint_move joint_move_5(n, "joint5_position_controller/");
    */
    // Changed this routine to add the Odrive joint_id a hardcoded value 
    joint_move joint_move_3(n, "joint3_position_controller/",  0);
    joint_move joint_move_4(n, "joint4_position_controller/",  1);
    joint_move joint_move_5(n, "joint5_position_controller/",  2);
    joint_move joint_move_2(n, "joint2_position_controller/",  3);


    /*####################here we are gonna add the call for l2 to initialised##################*/
    //joint_move joint_move_2(n, "joint2_position_controller/");   //edited by ribin this is the joint for l2
    /*##########################initialisation of l2 compleated################################*/

    joint_move::joint_pub_trajectory = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 2);
    joint_move::joint_homing_service = n.serviceClient<odrive_control::joint_homing>("/odrive_control/joint_init_to_home");
    //edited by ribin for making the l2 idle
    joint_move::joint_idle_service = n.serviceClient<odrive_control::joint_homing>("/odrive_control/joint_init_to_idle");
    /* Relay switches output */
    // Manohar Sambandam : 21 JAN 2023 - Commented out this ROS service, cotton detect through Yolo is triggered through a unix signal SIGUSR1, SIGUSR2
    //edited by ribin for calling cotton_detectin_service on 20_04_2020	
    //joint_move::cotton_detection_ml_service = n.serviceClient<cotton_detection::capture_cotton_srv>("/capture_cotton");	
    ROS_INFO("subscribed to capture");

    //arm_status_srv_ = n.advertiseService("/yanthra_move/current_arm_status", arm_status_function);


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
    // LED and Polariser working if Camera is enabled 
#if CAMERA_EN == true
    d_out led_control(n, "led_control");

#endif
    //Limit Switch I/P for Joint 5 Initialisation 
#if JOINT5_INIT_EN == true
    d_in joint5_switch_in(n, "joint5_switch_in");
    d_out joint5_init_start(n, "joint5_init_start");
#endif
    // START_SWITCH_IN and OUTPUT_INDICAATOR_LED for User to Start Picking operation 
#if START_SWITCH_EN == true
    d_in start_switch_in(n, "start_switch_in");
    d_out start_switch_out(n, "start_switch_out");
#endif
    a_out problem_indicator_out(n, "problem_indicator_out");  //user indicator
    // SHUTDOWN_SWITCH_IN, if start_stop switch is pressed for more than 5sec, initiate shutdown
#if SHUTDOWN_SWITCH_EN == true
    d_in shutdown_switch_in(n, "shutdown_switch_in");
#endif

    // INITIALISATION //


    ROS_WARN("yanthra_move :: Starting Initialisation\n\n");
    ROS_INFO("yanthra_move  Killing ros node cotton_detect_yolo") ;
    //system( "rosnode kill /cotton_detect_yolo") ;
    ROS_INFO(" yanthra_move :: Running cotton_detect_yolo program");
    //system( "~/pragati/start.sh") ;
    /* Cotton Coordinates */
    std::vector<geometry_msgs::Point> positions;
    std::vector<geometry_msgs::Point> positions3;
    std::vector<geometry_msgs::Point> positions4;
    std::vector<geometry_msgs::Point> positions5;
    std::vector<geometry_msgs::Point> positions5_origin;
    std::vector<geometry_msgs::PointStamped> positions_yanthra;
    std::vector<geometry_msgs::PointStamped> positions_link3;
    std::vector<geometry_msgs::PointStamped> positions_link4;
    std::vector<geometry_msgs::PointStamped> positions_link5;
    /*Adding link5_orgin in urdf to take tf between camera as well as yanthra to the link5_origin (Reason is link5 is variable length,we can't take tf wrt to that*/
    std::vector<geometry_msgs::PointStamped> positions_link5_origin;

    /* Find the transformed coordinates from camera_reference_frame to link3 */
    tf::StampedTransform tf_camera_base;
    tf::StampedTransform tf_yanthra_origin;
    tf::StampedTransform tf_link4_base;
    tf::StampedTransform tf_link5_base;
    tf::StampedTransform tf_link5_origin;
    tf::TransformListener YanthraListener;
    //tf::TransformListener listener_yanthra_origin;
    //tf::TransformListener listener_camera_base;
    //tf::TransformListener listener_link3_base ;
    //tf::TransformListener listener_link4_base ;
    //tf::TransformListener listener_link5_base;
    //tf::TransformListener listener_yanthra_origin_to_link3;
    //tf::TransformListener listener_yanthra_origin_to_link4;
    //tf::TransformListener listener_yanthra_origin_to_link5;
    //tf::TransformListener listener_yanthra_origin_to_yanthra;
        //YanthraListener.waitForTransform("/link3", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //YanthraListener.waitForTransform("/link4", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //YanthraListener.waitForTransform("/link5", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //YanthraListener.waitForTransform("yanthra_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));


#if CAMERA_EN == true
        // CAMERA_EN will be fale  for Test140 mode
        try {
        YanthraListener.waitForTransform("/link5", "/camera_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.waitForTransform("/link3", "/camera_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.waitForTransform("/link4", "/camera_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.lookupTransform("/link3", "/camera_link", ros::Time(0), tf_camera_base);
        YanthraListener.lookupTransform("/link4", "/camera_link", ros::Time(0), tf_link4_base);
        YanthraListener.lookupTransform("/link5", "/camera_link", ros::Time(0), tf_link5_base);
        YanthraListener.lookupTransform("/link5_origin", "/camera_link", ros::Time(0), tf_link5_origin);
        }
       catch(tf::TransformException ex)
       {
          ROS_INFO("CAMERA Enabled , transform to /camera_link, Error in Transform ");
          ROS_ERROR("%s", ex.what());
       }
       ROS_INFO("Difference between Camera and link5, x: %lf, y: %lf, z: %lf",
            tf_link5_base.getOrigin().x(),
            tf_link5_base.getOrigin().y(),
            tf_link5_base.getOrigin().z());
       ROS_INFO("Difference between Camera and link5_origin, x: %lf, y: %lf, z: %lf",
            tf_link5_origin.getOrigin().x(),
            tf_link5_origin.getOrigin().y(),
            tf_link5_origin.getOrigin().z());
       ROS_INFO("Difference between Camera and Link4, x: %lf, y: %lf, z: %lf",
            tf_link4_base.getOrigin().x(),
            tf_link4_base.getOrigin().y(),
            tf_link4_base.getOrigin().z());
       ROS_INFO("Difference between Camera and Link3, x: %lf, y: %lf, z: %lf",
            tf_camera_base.getOrigin().x(),
            tf_camera_base.getOrigin().y(),
            tf_camera_base.getOrigin().z());
#else
        try {
        YanthraListener.waitForTransform("/link5", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.waitForTransform("/link3", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.waitForTransform("/link4", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        YanthraListener.lookupTransform("/link3", "/yanthra_link", ros::Time(0), tf_camera_base);
        YanthraListener.lookupTransform("/link4", "/yanthra_link", ros::Time(0), tf_link4_base);
        YanthraListener.lookupTransform("/link5", "/yanthra_link", ros::Time(0), tf_link5_base);
        YanthraListener.lookupTransform("/link5_origin", "/yanthra_link", ros::Time(0), tf_link5_origin);
        }
       catch(tf::TransformException ex)
       {
          ROS_INFO("CAMERA Disabled , transform to /yanthra_link , Error in Transform ");
          ROS_ERROR("%s", ex.what());
       }
       ROS_INFO("Difference between Yanthra_link and link5, x: %lf, y: %lf, z: %lf",
            tf_link5_base.getOrigin().x(),
            tf_link5_base.getOrigin().y(),
            tf_link5_base.getOrigin().z());
       ROS_INFO("Difference between Yanthra_link and link5_origin, x: %lf, y: %lf, z: %lf",
            tf_link5_origin.getOrigin().x(),
            tf_link5_origin.getOrigin().y(),
            tf_link5_origin.getOrigin().z());
       ROS_INFO("Difference between Yanthra_link and Link4, x: %lf, y: %lf, z: %lf",
            tf_link4_base.getOrigin().x(),
            tf_link4_base.getOrigin().y(),
            tf_link4_base.getOrigin().z());
       ROS_INFO("Difference between Yanthra_link and Link3, x: %lf, y: %lf, z: %lf",
            tf_camera_base.getOrigin().x(),
            tf_camera_base.getOrigin().y(),
            tf_camera_base.getOrigin().z());
#endif 

        //listener_yanthra_origin_to_link3.waitForTransform("/link3", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        //listener_yanthra_origin_to_link4.waitForTransform("/link4", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        //listener_yanthra_origin_to_link5.waitForTransform("/link5", "/yanthra_link", ros::Time(0), ros::Duration(10.0));
        //listener_yanthra_origin.waitForTransform("yanthra_link", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //listener_camera_base.waitForTransform("/link3", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //listener_link4_base.waitForTransform("/link4", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //listener_link5_base.waitForTransform("/link5", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
        //listener_yanthra_origin.lookupTransform("/yanthra_link", "/camera_depth_optical_frame", ros::Time(0), tf_yanthra_origin);
        //listener_camera_base.lookupTransform("/link3", "/camera_depth_optical_frame", ros::Time(0), tf_camera_base);
        //listener_link4_base.lookupTransform("/link4", "/camera_depth_optical_frame", ros::Time(0), tf_link4_base);
        //listener_link5_base.lookupTransform("/link5", "/camera_depth_optical_frame", ros::Time(0), tf_link5_base);
        //YanthraListener.lookupTransform("/yanthra_link", "/camera_depth_optical_frame", ros::Time(0), tf_yanthra_origin);

    /* Wait for callback to be called */
    ros::Duration(5.0).sleep();



#if BREAD_BOARD == true
    return 0;
#endif
    std::string throwaway;
#if START_SWITCH_EN == true
    // start_switch_out.command(true); // not used
    //ROS_WARN("Initialisation complete, \n Press START_SWITCH to start the Robot...\n\n");
    bool STARTSWITCH = true;
    bool SHUTDOWNSWITCH = true;

    //test all hardware if required before go for picking

    //problem_indicator_out.command(USER_INPUT); // not used

    if(Global_vaccum_motor == true){
        ROS_WARN("TESTING THE HARDWARE");
        // vaccum_pump_test

        VacuumPump(false);
        ros::Duration(2.0).sleep();

        ROS_INFO("vaccum pump on");
        VacuumPump(true);
        ros::Duration(5.0).sleep();
        VacuumPump(false);
        ROS_INFO("vaccum pump off");
        
        // end_effector_test
        ROS_INFO("end effector on");
        SetEndEffectorDirection(CLOCKWISE) ;  // changed after the new board came on 19OCT2022
        EndEffector(true);
        ros::Duration(2.0).sleep();
        ROS_INFO("end effector off");
        EndEffector(false);

        ROS_INFO("end effector on");
        SetEndEffectorDirection(ANTICLOCKWISE);
        EndEffector(true);
        ros::Duration(2.0).sleep();
        ROS_INFO("end effector off");
        EndEffector(false);
        ros::Duration(2.0).sleep();
        EndEffectorDrop(DROP_EEF);
        ros::Duration(2.0).sleep();
        EndEffectorDrop(STOP_EEF);

        // camera_led_test
        ROS_INFO("camera led on");
        camera_led(true);
        ros::Duration(2.0).sleep();
        ROS_INFO("camera led off");
        camera_led(false);
        ros::Duration(2.0).sleep();
        //cotton drop shutter test
        ROS_INFO("shutter open");
	cotton_drop_solenoid_shutter();
        //cotton_drop_shutter();
        ros::Duration(2.0).sleep();
        ROS_INFO("shutter close");
    }

    // system is readt for initialisation turn on green led
    green_led_on();

    ROS_WARN("Initialisation complete, \n Press START_SWITCH to start the Robot...\n\n");

    //Updating the arm status , false for NOT READY, true for READY
    arm_status = "ready";
    //ROS_INFO(" Arm_status : %s",arm_status);
    if (ARM_CALIBRATION == true){
        ROS_WARN("Initialisation complete, \n Press ENTER to start the Robot...(type 'return' to terminate the process)\n\n");
        std::getline(std::cin, throwaway);
        if(throwaway.compare("return") == 0)
            return 0;

    }
    else{	
	    //waitForStartSwitch() ;
	    // TODO : add this code and remove code below CheckForShutDownSwitchAndShutDownSystem
	    //CheckForShutDownSwitchAndShutDownSystem();
	    while((start_switch_in.state() == false))// && (start_switch_status() == false))
	    {	

		    ros::spinOnce();
		    ros::Duration(0.1).sleep();
		    if((shutdown_switch_in.state() == true || shutdown_switch_status() == true )){
			    ROS_ERROR("SHUTDOWN Switch ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM ");
			    blink_led_on();
			    ros::Duration(2).sleep();
			    if (save_logs == true){
				    ROS_WARN("copying all the logs");
				    system("/home/ubuntu/pragati/launch_files/copy.sh");
			    }

			    // TODO odrive_hw_controller.ShutDown();

			    //problem_indicator_out.command(SHUTDOWN);
			    ROS_ERROR("SHUTDOWN SWITCH ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM");
			    system("rosnode kill -a");
			    system("sudo -S shutdown -P now");
			    red_led_on();
			    return(false) ;
		    }
	    }

	    red_led_on();
	    //problem_indicator_out.command(INPROCESS);
	    //start_switch_out.command(false);
    }
    //Updating the arm status , false for NOT READY, true for READY
    arm_status = "busy";
    //turn on the red led on
    red_led_on();

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

    // Manohar Sambandam : 21 JAN 2023 - Commented out this ROS service, cotton detect through Yolo is triggered through a unix signal SIGUSR1, SIGUSR2
    //edited by ribin for checking the cotton_detection_server
    //cotton_detection::capture_cotton_srv cotton_detection_srv;


    //added by ribin for changing the sequance to height scan first
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
    if (height_scan_enable ==true) {
        srv.request.joint_id = 3;
        if(joint_move::joint_homing_service.call(srv)!=true)
        {
            ROS_ERROR("Joint2 Homing, Reason: %s", srv.response.reason.c_str());
            problem_indicator_out.command(ERROR);
            return 0;
        }
        srv_idle.request.joint_id = 3;//TODO:Joint2_Idle_Removing_To_Stop_Moving
        ros::Duration(l2_idle_sleep_time).sleep();
        //ros::Duration(20).sleep();
        if(joint_move::joint_idle_service.call(srv_idle)!=true) //TODO:Joint2_Idle_Removing_To_Stop_Moving
        {
            ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
            problem_indicator_out.command(ERROR);
            return 0;
        }


    }
    //>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // Joint5 Homing (r, i.e. Prismatic Joint)
    // TODO convert hard coded nos to Joint5ID ..
    srv.request.joint_id = 2;
    if(joint_move::joint_homing_service.call(srv) != true)
    {
        ROS_ERROR("Joint5 Homing, Reason: %s", srv.response.reason.c_str());
        problem_indicator_out.command(ERROR);
        return 0;
    }

    // Joint3 Homing (phi, i.e. Revolute Joint sweeping along vertical)
    // TODO convert hard coded nos to Joint3ID ..
    srv.request.joint_id = 0;
    if(joint_move::joint_homing_service.call(srv) != true)
    {
        ROS_ERROR("Joint3 Homing, Reason: %s", srv.response.reason.c_str());
        problem_indicator_out.command(ERROR);
        return 0;
    }

    // Joint4 Homing (theta, i.e. Revolute Joint sweeping along horizontal)
    // TODO convert hard coded nos to Joint4ID ..
    srv.request.joint_id = 1;
    if(joint_move::joint_homing_service.call(srv) != true)
    {
        ROS_ERROR("Joint4 Homing, Reason: %s", srv.response.reason.c_str());
        problem_indicator_out.command(ERROR);
        return 0;
    }
    //chaned by ribin on 10/03/2020
    //Changing the sequance of initialisation to l2 first so moved this poriton of the code first


    ROS_INFO("All joints are initialised");

    /* Move Joint3 and Joint4 to their default pose */
    //joint_move_3.move_joint(0.0, WAIT);
    //joint_move_4.move_joint(0.0, WAIT);

    /* While ROS OK */
    while(ros::ok())
    {
      // Stop the PublisherThread by settng the variable 

      //PublisherThread = false ;

      // Call this routine to publish the joint_states
      // the topic /joint_states is used by robot_state_publisher to publish the dynamice tf values
      // It is also used by other tools like the calibration tool to find the status of each joint at any point of time
       //odrive_hw_interface->read(elapsed_time );
      //odrive_hw_interface->write(elapsed_time );
#if START_SWITCH_EN == true
        ROS_WARN("Ready to go to another cycle, \n Press START_SWITCH start the Robot...\n\n");
        //start_switch_out.command(true); // Switch On the Red bulb indicator
        //problem_indicator_out.command(USER_INPUT);
        //ROS_WARN("%d",STARTSWITCH);
        //ROS_WARN("%d",start_switch_in.state());
        green_led_on();

        //Updating the arm status , false for NOT READY, true for READY
        arm_status = "ready";
        //while(  (!(SHUTDOWNSWITCH=shutdown_switch_in.state())) && (!(STARTSWITCH = start_switch_in_state())) )
        if (ARM_CALIBRATION == true){
            ROS_WARN("calibration completed, \n Press ENTER to start the Robot...(type 'return' to terminate the process)\n\n");
            std::getline(std::cin, throwaway);
            if(throwaway.compare("return") == 0)
                return 0;

        }
        else{	
            //waitForStartSwitch() ;
            // TODO : add this code and remove code below CheckForShutDownSwitchAndShutDownSystem
            //CheckForShutDownSwitchAndShutDownSystem();
	    while((start_switch_in.state() == false))// && (start_switch_status() == false))
	    {	

		    ros::spinOnce();
		    ros::Duration(0.1).sleep();
		    if((shutdown_switch_in.state() == true || shutdown_switch_status() == true )){
			    ROS_ERROR("SHUTDOWN Switch ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM ");
			    blink_led_on();
			    ros::Duration(2).sleep();
			    if (save_logs == true){
				    ROS_WARN("copying all the logs");
				    system("/home/ubuntu/pragati/launch_files/copy.sh");
			    }

			    // TODO odrive_hw_controller.ShutDown();

			    //problem_indicator_out.command(SHUTDOWN);
			    ROS_ERROR("SHUTDOWN SWITCH ACTIVATED :: SHUTTING  DOWN ROS AND SYSTEM");
//TODO:Add code to shutdown odrive Before shutdown the system
			    system("rosnode kill -a");
			    system("sudo -S shutdown -P now");
			    red_led_on();
			    return(false) ;
		    }
	    }
        }
        red_led_on();

        //Updating the arm status , false for NOT READY, true for READY
        arm_status = "busy";

        ROS_INFO( "Got STARTSWITCH COMMAND Continuing the program ... \n") ;
#else
        ROS_WARN("Ready to go to another cycle, \n Press ENTER to start the Robot...(type 'return' to terminate the process)\n\n");
        std::getline(std::cin, throwaway);
        if(throwaway.compare("return") == 0)
            return 0;
#endif

        //problem_indicator_out.command(INPROCESS);

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
            ROS_INFO("Copying all the files to debug directory");
            system("cp /home/ubuntu/pragati/outputs/DetectionOutput.jpg /home/ubuntu/pragati/debug/$(date +%Y-%m-%d_%H:%M:%S)_DetectionOutput.jpg");
            system("cp /home/ubuntu/pragati/inputs/img100.jpg /home/ubuntu/pragati/debug/$(date +%Y-%m-%d_%H:%M:%S)_img100.jpg");
            if (Trigger_Camera==true){
                       //copy_command.str("");
                       //copy_command << "mv " YANTHRA_DATA_OUTPUT_DIR "/DetectionOutput.jpg " << path + "_debug/" << datecode << "_DetectionOutput.jpg";
                       //system(copy_command.str().c_str());
                       ROS_INFO("Removing all the files in inputs and output Directory");
//TODO:Cleanup System Call
                       system( "rm /home/ubuntu/pragati/inputs/img100.jpg");
                       system( "rm /home/ubuntu/pragati/outputs/CottonCoordinatesSorted.txt");
                       system( "rm /home/ubuntu/pragati/outputs/CottonCoordinatesUnsorted.txt");
                       system( "rm /home/ubuntu/pragati/outputs/cotton_details.txt");
            }
            double joint2_pose =  joint_pose_values[i][0];
            double joint3_pose  = joint_pose_values[i][1];
            double joint4_pose  = joint_pose_values[i][2];
            double joint5_pose  = joint_pose_values[i][3]; // added to standardise with other joints
            ROS_INFO("link2 poses %f:=",joint2_pose);
            ROS_INFO("link3 poses %f:=",joint3_pose);
            ROS_INFO("link4 poses %f:=",joint4_pose);
            ROS_INFO("link5 poses %f:=",joint5_pose);
            ROS_INFO("link2 old poses %f:=",joint2_old_pose);
            
            if (height_scan_enable ==true) {
                if (joint2_pose < height_scan_max)
                {
                    if(joint2_old_pose != joint2_pose){


                        //joint_move_4.move_joint(joint4_homing_position, WAIT); // TODO move it to joint4_homing_position
                        //ros::Duration(2).sleep();
                        //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();

                        joint_move_2.move_joint(joint2_pose,WAIT);
                        ros::Duration(l2_step_sleep_time).sleep();
                        ROS_INFO("height_scan_step value := %f", joint2_pose);
                        ROS_INFO("l2 sleep time := %f", l2_step_sleep_time);
                        srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
                        ros::Duration(l2_idle_sleep_time).sleep();
                        if(joint_move::joint_idle_service.call(srv_idle)!=true)
                        {
                            ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
                            problem_indicator_out.command(true);
                            return 0;
                        }//TODO:Joint2_Idle_Removing_To_Stop_Moving*/
                        //		ROS_INFO("height_scan_value %f:=", height_scan_value);



                    }
                    joint2_old_pose = joint2_pose;
                }

            }


            //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
            joint_move_5.move_joint(joint5_pose, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
            // TODO : remove the following Sleep. Currently it may create violent motion of the arm.
            ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
            //ros::Duration(2).sleep();
            joint_move_3.move_joint(joint3_pose,WAIT);
            //ros::Duration(2).sleep();
            joint_move_4.move_joint(joint4_pose, WAIT);
            ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION).sleep();


            // if we are running calibration , read the cotton file only here , camera not used
            if (ARM_CALIBRATION == true){

                ROS_WARN(" Yanthra_move :: ARM CALIBRATION RUNNING");
                ROS_INFO("Reading  the coordinates");
                get_cotton_coordinates(&positions);

                //getCottonCoordinates_ToYanthraOrigin(&positions, &positions_yanthra);
                //getCottonCoordinates_ToLink3(&positions, &positions_link3);
                //getCottonCoordinates_ToLink4(&positions, &positions_link4);
                //getCottonCoordinates_ToLink5(&positions, &positions_link5);
                getCottonCoordinates_yanthra_origin_to_yanthra(&YanthraListener, &positions,&positions_yanthra);
                getCottonCoordinates_yanthra_origin_to_link3(&YanthraListener, &positions, &positions_link3);
                getCottonCoordinates_yanthra_origin_to_link4(&YanthraListener, &positions, &positions_link4);
                getCottonCoordinates_yanthra_origin_to_link5(&YanthraListener, &positions, &positions_link5);
                getCottonCoordinates_yanthra_origin_to_link5_origin(&YanthraListener, &positions, &positions_link5_origin);
                /*
                getCottonCoordinates_yanthra_origin_to_yanthra(&listener_yanthra_origin_to_yanthra, &positions,&positions_yanthra);
                getCottonCoordinates_yanthra_origin_to_link3(&listener_yanthra_origin_to_link3, &positions, &positions_link3);
                getCottonCoordinates_yanthra_origin_to_link4(&listener_yanthra_origin_to_link4, &positions, &positions_link4);
                getCottonCoordinates_yanthra_origin_to_link5(&listener_yanthra_origin_to_link5, &positions, &positions_link5);
                */
            }
            else{
                // NORMAL OPERATION
                /* Initiate a camera start request */
                ROS_WARN("YanthraLabCalibrationTesting_1787: %d",YanthraLabCalibrationTesting); //edited on dec_18 for testing
                if (!YanthraLabCalibrationTesting) {
#if CAMERA_EN == true
                    ROS_WARN("YanthraLabCalibrationTesting_inside: %d",YanthraLabCalibrationTesting);
                    // NORMAL OPERATION CAMERA_EN is TRUE and  YanthraLabCalibration is FALSE
                    /* Initiate a camera start request */
                    //ros::Duration(3).sleep();
                    //led_control.command(START);
                    // turn on led

                    //camera_led(true);
                    //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
                    //vaccum_motor.command(START); //added by ribin for controlling the blower motor
                    /*
                    ROS_INFO("Executing new program: " COTTON_DETECT_PROGRAM " " YANTHRA_DATA_INPUT_DIR "/ " YANTHRA_DATA_OUTPUT_DIR "/");
                    cotton_detection_srv.request.capture = true;
                    ros::Duration(2).sleep();
                    */

                    // edited by ribin for converting the system call to service call
                    /*
                       if(joint_move::cotton_detection_ml_service.call(cotton_detection_srv) != true)

                       {
                       ROS_INFO("cotton_detection_service called");

                       }
                       */
                    
                    /*
                    ROS_INFO("cotton detection service started");
                    if(joint_move::cotton_detection_ml_service.call(cotton_detection_srv) != true)
                    {
                        problem_indicator_out.command(USER_INPUT);
                        ROS_ERROR("Cotton Detection Failed: ");
                        system("/home/ubuntu/pragati/camera_rs2_recovery.sh");
                        ROS_WARN("Recovering From rs2 error");
                        ros::Duration(10).sleep(); // This large time is given to help from the RS2_error of the camera
                        //ros::Duration(CottonCaptureDetectWaitTime).sleep();

                    }
                    ROS_INFO("cotton detection service finished");
                    */


                    /*	if(state != 0)
                        {
                        ROS_ERROR("Cotton Detection Failed: %d", state);
                        problem_indicator_out.command(true);
                        }
                        */
                    //led_control.command(STOP);
                    //
                    ROS_WARN("Waiting for the Arm to Move and Stablize");
                    //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
                    ros::Duration(4).sleep();
		    //Code Added for OAK-Lite Camera 
                    if (Trigger_Camera==true){
                    ROS_WARN("Triggering Capture and Detection");
                    camera_led(true);
                    TriggerCaptureAndDetectCotton();
                    }
                    //TODO Add CameraFailure Mode code
                    camera_led(false);
                    record_debug_data();		// Store the captured images in debug folder
#else

                    //#elif TEST140MODE == true
                    // This is TEST140 mode 
                    /* Get the coordinates of cotton */
                    ROS_INFO("In TEST140 , Not in YanthraCalibration or ARUCO : Reading the coordinates files");
                    get_cotton_coordinates(&positions);
                    // Of TEST140 mode
                    //#endif	

#endif
                    // Read the cotton coordinates
                    ROS_INFO("yanthra_move :: read the cotton points");
                    get_cotton_coordinates(&positions);

                    /* ikkConvert cotton coordinates from camera_frame to link3 frame */
#if CAMERA_EN == true
                    //TODO : Convert listener_camera_base to listener_link5_base
                    getCottonCoordinates_cameraToYanthraOrigin(&YanthraListener, &positions,&positions_yanthra);
                    getCottonCoordinates_cameraToLink3(&YanthraListener,&positions, &positions_link3);
                    getCottonCoordinates_cameraToLink4(&YanthraListener, &positions, &positions_link4);
                    getCottonCoordinates_cameraToLink5(&YanthraListener, &positions, &positions_link5);
                    getCottonCoordinates_cameraToLink5Origin(&YanthraListener, &positions, &positions_link5_origin);
                    /*
                    getCottonCoordinates_cameraToYanthraOrigin(&listener_yanthra_origin, &positions,&positions_yanthra);
                    getCottonCoordinates_cameraToLink3(&listener_camera_base,&positions, &positions_link3);
                    getCottonCoordinates_cameraToLink4(&listener_link4_base, &positions, &positions_link4);
                    getCottonCoordinates_cameraToLink5(&listener_link5_base, &positions, &positions_link5);
                    */
#else
                    //getCottonCoordinates_ToYanthraOrigin(&positions, &positions_yanthra);
                    //getCottonCoordinates_ToLink3(&positions, &positions_link3);
                    //getCottonCoordinates_ToLink4(&positions, &positions_link4);
                    //getCottonCoordinates_ToLink5(&positions, &positions_link5);

                    getCottonCoordinates_yanthra_origin_to_yanthra(&YanthraListener, &positions,&positions_yanthra);
                    getCottonCoordinates_yanthra_origin_to_link3(&YanthraListener, &positions, &positions_link3);
                    getCottonCoordinates_yanthra_origin_to_link4(&YanthraListener, &positions, &positions_link4);
                    getCottonCoordinates_yanthra_origin_to_link5(&YanthraListener, &positions, &positions_link5);
                    getCottonCoordinates_yanthra_origin_to_link5_origin(&YanthraListener, &positions, &positions_link5_origin);
#endif
                }
                else { // of YanthraLabCalibrationTesting

                    ROS_WARN("yanthra move doing  YanthraLabCalibrationTesting ");
                    std::string cotton_coordinate_filename;
                    //strcpy(cotton_coordinate_filename, PRAGATI_INSTALL_DIR) ;
                    //TODO	change all PRAGATI to simple strings
                    PRAGATI_INSTALL_DIR = "/home/ubuntu/pragati/" ;
                    cotton_coordinate_filename = PRAGATI_INSTALL_DIR + "/outputs/cotton_details.txt";
                    //strcpy(FilePathName,NameListFilePath.c_str());

                    //sprintf(FilePathName ,"%s/cotton_details.txt", PRAGATI_OUTPUT_DIR);
                    //strncpy( cotton_coordinate_filename , FilePathName ); 
                    //cotton_coordinate_filename = FilePathName ;	

                    // Call ARUCO_FINDER_PROGRAM
                    // Aruco_Finder put the location of the files in the file centroid.tx
                    // Read data from centroid.txt and then move the arm
                    std::cout << " Yanthra_Move :: Deleting old centroid.txt file if it exists" << std::endl ;
                    system(" echo /bin/rm  /home/ubuntu/.ros/centroid.txt ");
                    system("rm /home/ubuntu/input/points100.pcd ");
                    system("rm /home/ubuntu/input/img100.jpg") ;        
                    if (system("/bin/rm  /home/ubuntu/.ros/centroid.txt")) {
                        std::cerr << " aruco_finder :: Not able to delete /home/ubuntu/.ros/centroid.txt /home/ubuntu/input/points100.pcd /home/ubuntu/input/img100.jpg " << std::endl ;
                    } 
                    else {
                        system ("ls -l /home/ubuntu/.ros ; date ") ;
                    } 
                    //system(ARUCO_FINDER_PROGRAM);
                    if (system(ARUCO_FINDER_PROGRAM) != 0) 
                    {
                        ROS_ERROR("yanthra_move :: ARUCO_FINDER PROGRAM FAILED") ;
                        //ROS_ERROR ("yanthra_move :: EXITING FROM yanthra_move ");
                        ROS_ERROR ("yanthra_move :: relaunching aruco after 4s ");
                        //exit(1) ;
                        //ros::Duration(4).sleep();
                    }

                    /* Read  from the generated centroid.txt file from aruco_finder */
                    /* centroid.txt is expected at ~/.ros/centroid.txt */
                    std::ifstream mark_centroid("/home/ubuntu/.ros/centroid.txt");
                    if (!mark_centroid.is_open()) {
                        ROS_INFO("Error: not able to open the file : centroid.txt ");
                        ROS_INFO("Exiting yanthra_move ");
                        ROS_ERROR("Error: not able to open the file : centroid.txt ");
                        ROS_ERROR("Exiting yanthra_move ");
                        //exit(1) ;
                    }

                    cotton_coordinate_filename = "/home/ubuntu/pragati/outputs/cotton_details.txt";
                    std::ofstream cotton_fs(cotton_coordinate_filename.c_str());

                    if (!cotton_fs.is_open()) {
                        ROS_INFO("Error: not able to open the file : %s ", cotton_coordinate_filename.c_str());
                        ROS_INFO("Exiting yanthra_move ");
                        ROS_ERROR("Error: not able to open the file : %s ", cotton_coordinate_filename.c_str());
                        ROS_ERROR("Exiting yanthra_move ");
                        //exit(1) ;
                    }


                    float XValue, YValue, ZValue;
                    int PixelColumn = 0 ;
                    int PixelRow = 0 ;

                    while(mark_centroid >> XValue >> YValue >> ZValue)
                    {
                        // Print to the cotton_details.txt file the four corners of the marker
                        // These values are w.r.t to the Camera origin.
                        // When using in the arm this has to be converted to ARM's origin.

                        cotton_fs << PixelColumn <<" " << PixelRow << " " <<" "<< (float) XValue << " " <<(float)YValue << " " << (float) ZValue << std::endl ;
                        cotton_fs.flush();
                        ROS_WARN( " ArucoMarker Values :  XValue : %.4f, YValue :  %.4f, ZValue : %.4f", XValue, YValue, ZValue) ;
                        writeFormattedMessageToLogFile(log_filename, 
                        " ArucoMarker Values :  XValue : %.4f, YValue :  %.4f, ZValue : %.4f\n", XValue, YValue, ZValue);                   
                      }
                    // Close the output steam
                    cotton_fs.close();
                    mark_centroid.close();

                    // Reading the CottonCoordinate from a file
                    ROS_WARN(" ARUCO Marker Test : printing cotton coordinates from cotton_details.txt");
                    get_cotton_coordinates(&positions);
                    //for(int i=0; i < positions.size(); i++)
                        //ROS_WARN("%f", positions.at(i));
                    getCottonCoordinates_cameraToYanthraOrigin(&YanthraListener, &positions, &positions_yanthra);
                    getCottonCoordinates_cameraToLink3(&YanthraListener, &positions, &positions_link3);
                    getCottonCoordinates_cameraToLink4(&YanthraListener, &positions, &positions_link4);
                    getCottonCoordinates_cameraToLink5(&YanthraListener, &positions, &positions_link5);
                    getCottonCoordinates_cameraToLink5Origin(&YanthraListener, &positions, &positions_link5_origin);
                    /*
                    getCottonCoordinates_cameraToYanthraOrigin(&listener_camera_base, &positions, &positions_yanthra);
                    getCottonCoordinates_cameraToLink3(&listener_camera_base, &positions, &positions_link3);
                    getCottonCoordinates_cameraToLink4(&listener_link4_base, &positions, &positions_link4);
                    getCottonCoordinates_cameraToLink5(&listener_link5_base, &positions, &positions_link5);
                    */
                    ROS_WARN("printing cotton coordinates positions_yanthra");
                    //for(int i=0; i < positions_yanthra.size(); i++)
                        //ROS_WARN("Camera Coordinates : %f",  positions_yanthra.at(i));


                    ROS_INFO("printing cotton coordinates positions_link3");
                    //for(int i=0; i < positions_yanthra.size(); i++)
                        //ROS_WARN("LINK3 POSITIONS : %f", positions_link3.at(i));

                    ROS_INFO("printing cotton coordinates positions_link4");
                    //for(int i=0; i < positions_yanthra.size(); i++)
                        //ROS_WARN("LINK4 POSITIONS : %f", positions_link4.at(i));

                    ROS_INFO("printing cotton coordinates positions_link5");
                    //for(int i=0; i < positions_yanthra.size(); i++)
                        //ROS_WARN("LINK5 POSITIONS %f",  positions_link5.at(i));

                } //  of -- if (YanthraLabCalibration) 
            }
            if(Global_vaccum_motor == true)
            {
                //vaccum_motor.command(START); //added by ribin for controlling the blower motor
                //TODO check whether the motor is still on or not



            }
            // here we are gonna pick all cotton in current pose, the positions_link3 variable is loaded with positions of each cotton which is validated for reachability 
            int trgt = 0;
            //	int picked =0;
            double  start_time_to_pick_n_number_of_cotton = currentTimeMillis();

            // Added to check if the no of Positions is non-zero
            int NoOfTargetLocations = 0 ;
            NoOfTargetLocations = positions_yanthra.size() ;
            ROS_WARN("yanthra_move :: NotOfCottonLocations %d", NoOfTargetLocations );
            if ( NoOfTargetLocations > 0 ) {
                // Move all the joints to the base home posistion first to start from a known position
                //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                //ros::Duration(0.1).sleep();
                //joint_move_3.move_joint(joint3_pose,WAIT);
                //ros::Duration(0.1).sleep();
                //joint_move_4.move_joint(joint4_pose, WAIT);
                //ros::Duration(0.1).sleep();
                //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();

                if(Global_vaccum_motor == true){
                        VacuumPump(true);
                }
                // Move through each and every location in the positions_yanthra list
                for(std::vector<geometry_msgs::PointStamped>::iterator it = positions_yanthra.begin(),
                        /*std::vector<geometry_msgs::PointStamped>::iterator*/ it_local3 = positions_link3.begin(),
                        /*std::vector<geometry_msgs::PointStamped>::iterator*/ it_local4 = positions_link4.begin(),
                        /*std::vector<geometry_msgs::PointStamped>::iterator*/ it_local5 = positions_link5.begin(),
                        /*std::vector<geometry_msgs::PointStamped>::iterator*/ it_local5_origin = positions_link5_origin.begin();
			///*std::vector<geometry_msgs::PointStamped>::iterator*/ it_CameraPosition = positions.begin();
                        it != positions_yanthra.end();
                        //it++, it_local4++, it_local3++, it_local5++, it_CameraPosition++)
			it++, it_local4++, it_local3++, it_local5++, it_local5_origin++)
                {


                    //set_servo_pulsewidth(pi,vaccum_motor_pin,vaccum_on); // VACCUM MOTOR ON

                    // r, theta , phi is w.r.t to Link5 which controls Z of the ARM
                    // Link3 controls the Y(PHI) of the arm
                    // Link4 controls the X(THETA)  of the arm
                    double	rYanthra = 0,
                            thetaYanthra = 0,
                            phiYanthra = 0;
                    double	rLink3 = 0,
                            thetaLink3 = 0,
                            phiLink3 = 0;
                    double	rLink4 = 0,
                            thetaLink4 = 0,
                            phiLink4 = 0;
                    double	rLink5 = 0,
                            thetaLink5 = 0,
                            phiLink5 = 0;
                    double      rLink5_origin = 0,
                            thetaLink5_origin = 0,
                            phiLink5_origin = 0;
                    double	rCamera = 0,
                            thetaCamera = 0,
                            phiCamera = 0;

                    /* If targets to be reached in steps, rather continous operation */
                    if(continous_operation != true)
                    {
                        ROS_WARN("Press ENTER to move to next target...(type 'done' to terminate the process)\n\n");
                        std::getline(std::cin, throwaway);
                        if(throwaway.compare("done") == 0)
                            break;
                    }
                    // Commented the following 2 lines - 24AUG2023 they are redundant moves
                    //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                    //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
                    // turn off end effector before next picking
                    if(Global_vaccum_motor == true){
                        VacuumPump(true);
                    }

                    //ros::Duration(2).sleep();
                    //joint_move_3.move_joint(joint3_pose,WAIT);
                    //ros::Duration(2).sleep();
                    //joint_move_4.move_joint(joint4_pose, WAIT);
                    //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
                    // Convert the coordinates to L4 and L3 
                    // X coordinate is the value at link4 
                    // Y is the value at Link3 	
                    // Z is the value at Link5
                    //std::vector<geometry_msgs::Point> Link3,Link4,Link5;	
                    //std::vector<geometry_msgs::PointStamped> positions_Link3;
                    //std::vector<geometry_msgs::PointStamped> positions_Link4;		
                    //std::vector<geometry_msgs::PointStamped> positions_Link5;



                    ROS_WARN("Moving to target NO  %d\n", ++trgt);
                    /* Convert XYZ coordinate to polar coordinates */
                    // TODO : use tf transformed coordinates to convert XYZ in FLU (REP103) to R, Theta, PHI  currently we are using xyz_to_polar which is a patch
                    // TODO use tf::Pose::getRollPitchYaw(pose,roll,pitch,yaw); instead of xyz_to_polar function
                    // Incoming coordinate is in FLU
                    // TODO:xyztopolar uses RDF now, so while calling xyz_to_polar pass x , y ,z of (FLU ) to z, x, y (of RDF) [We are passing without the Sine Conversion,Have to Validate]
                    // then use the resultant  Phi to Link3
                    // then use the resultant  Theta to Link4
                    // then use the resultant R to Link5   
                    //xyz_to_polar(-(it->point.y), -(it->point.z), it->point.x, &rYanthra, &thetaYanthra, &phiYanthra);
                    //xyz_to_polar(-(it_local3->point.y), -(it_local3->point.z), it_local3->point.x, &rLink3, &thetaLink3, &phiLink3);
                    //xyz_to_polar(-(it_local4->point.y), -(it_local4->point.z), it_local4->point.x, &rLink4, &thetaLink4, &phiLink4);
                    //xyz_to_polar(-(it_local5->point.y), -(it_local5->point.z), it_local5->point.x, &rLink5, &thetaLink5, &phiLink5);
                    ROS_WARN("INPUT VALUE TO FORMULA FOR YANTHRA to YANTHRA  X : %f Y : %f Z : %f ",it->point.x, it->point.y, it->point.z) ;
                    ROS_WARN("INPUT VALUE TO FORMULA FOR LINK3 to YANTHRA X : %f Y : %f Z : %f ",it_local3->point.x, it_local3->point.y, it_local3->point.z) ;
                    ROS_WARN("INPUT VALUE TO FORMULA FOR LINK4 to YANTHRA X : %f Y : %f Z : %f ",it_local4->point.x, it_local4->point.y, it_local4->point.z) ;
                    ROS_WARN("INPUT VALUE TO FORMULA FOR LINK5 to YANTHRA X : %f Y : %f Z : %f ",it_local5->point.x, it_local5->point.y, it_local5->point.z) ;
                    ROS_WARN("INPUT VALUE TO FORMULA FOR LINK5_ORIGIN to YANTHRA X : %f Y : %f Z : %f ",it_local5_origin->point.x, it_local5_origin->point.y, it_local5_origin->point.z) ;

                    ConvertXYZToPolarFLUROSCoordinates((it->point.x), (it->point.y), it->point.z, &rYanthra, &thetaYanthra, &phiYanthra);
                    ConvertXYZToPolarFLUROSCoordinates((it_local3->point.x), (it_local3->point.y), it_local3->point.z, &rLink3, &thetaLink3, &phiLink3);
                    ConvertXYZToPolarFLUROSCoordinates((it_local4->point.x), (it_local4->point.y), it_local4->point.z, &rLink4, &thetaLink4, &phiLink4);
                    ConvertXYZToPolarFLUROSCoordinates((it_local5->point.x), (it_local5->point.y), it_local5->point.z, &rLink5, &thetaLink5, &phiLink5);
                    ConvertXYZToPolarFLUROSCoordinates((it_local5_origin->point.x), (it_local5_origin->point.y), it_local5_origin->point.z, &rLink5_origin, &thetaLink5_origin, &phiLink5_origin);
                    //TODO:MR HACK FOR rLink5,rLink5 is aslways positvie even if point x is negative  so we are converting all coordinate from origin of link5
                    //CURRENT HACK using it_local5->point.x using as rLink5
                    //TODO:We have to all the tf wrt origin of the frames,We have to change to Link5 now(already doing it for Link3 and Link5)
                    //rLink5=it_local5->point.x;
                    ROS_WARN("rYanthra: %f thetaYanthra : %f phiYanthra : %f ",rYanthra, thetaYanthra, phiYanthra) ;
                    ROS_WARN("rLink3  : %f thetaLink3   : %f phiLink3   : %f ",rLink3, thetaLink3, phiLink3) ;
                    ROS_WARN("rLink4  : %f thetaLink4   : %f phiLink4   : %f ",rLink4, thetaLink4, phiLink4) ;
                    ROS_WARN("rLink5  : %f thetaLink5   : %f phiLink5   : %f ",rLink5, thetaLink5, phiLink5) ;
                    ROS_WARN("rLink5_origin  : %f thetaLink5_origin   : %f phiLink5_origin   : %f ",rLink5_origin, thetaLink5_origin, phiLink5_origin) ;
                    ROS_WARN( " In yanthra_move.cpp") ;

                    ROS_WARN( " YANTHRA Move Value wrt YANTHRA : RYanthra : %f, thetaYanthra : %f , phiYanthra :%f ",rYanthra, thetaYanthra, phiYanthra) ;
                    ROS_WARN( "YANTHRA Move Value wrt  LINK3 : RLINK3 : %f, THETALINK3 : %f ,PHILINK3 :%f ",rLink3,thetaLink3, phiLink3) ;
                    ROS_WARN( "YANTHRA Move Value wrt  LINK4 : RLINK4 : %f, THETALINK4 : %f ,PHILINK4 :%f ",rLink4,thetaLink4, phiLink4) ;
                    ROS_WARN( "YANTHRA Move Value wrt  LINK5 : RLINK5 : %f, THETALINK5 : %f ,PHILINK5 :%f ",rLink5,thetaLink5, phiLink5) ;
                    ROS_WARN( "YANTHRA Move Value wrt  LINK5_ORIGIN : RLINK5_ORIGIN : %f, THETALINK5_ORIGIN : %f ,PHILINK5 :%f ",rLink5_origin,thetaLink5_origin, phiLink5_origin) ;
                     //TODO:by MR To Verify Mirror in Y
                    //phiLink3=phiLink3*(-1);
                    /*The Reason to pass this Value is to account for all the joint errors,This value should be equal to Yanthra Origin Values */
                    ROS_INFO( "We are Passing the Value of  RLINK5_ORIGIN : %f, THETALINK4 : %f ,PHILINK3 :%f to Controller",rLink5_origin,thetaLink4, phiLink3) ;
                    ROS_WARN( "Difference between Yanthra Origin Values and the Individual Joint Values :DeltaR : %f, DeltaTheta : %f ,DeltaPhi :%f ", rYanthra-rLink5_origin,thetaYanthra-thetaLink4,phiYanthra-phiLink3) ;

                    // Check Reability first and Check Field of View for BOTH theta and Phi values
                    ROS_INFO ("Checking Reachability for the target wrt to Yanthra Origin") ;
                    if(check_reachability(rYanthra, thetaYanthra, phiYanthra) == true)
                        // && (abs(thetaCamera) <= FOVThetaMax)) && (abs(PhiCamera<= FOVPhiMax))
                        {
                            //ros::Duration(4).sleep();
                            double picking_time_started = currentTimeMillis();
                            //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                            float joint5_delay = (rYanthra/joint5_vel_limit);
                            //ros::Duration(joint5_delay).sleep();
                            ROS_INFO("joint5 return delay: %f ", joint5_delay);
                            ROS_INFO("picked number: %d ", picked);
                            ROS_WARN( "YANTHRA_MOVING_TO_THE_TARGET_POSITION_WHICH_IS_AT RLINK5_ORIGIN : %f, THETALINK4 : %f ,PHILINK3 :%f ",rLink5_origin,thetaLink4, phiLink3) ;

                          joint_move_4.move_joint(thetaLink4, WAIT);
                          ROS_WARN("\n\n thetalink4 value is : %f",joint4_pose + thetaLink4);
                          ROS_WARN("\n joint4_pose value is  : %f",joint4_pose);
                          joint_move_3.move_joint(joint3_pose + phiLink3 ,WAIT);
                          ROS_WARN("\n joint3_pose:%f",joint3_pose);
                          ROS_WARN("phiLink3 : %f ",phiLink3);
//By Mani Radhakrishnan:Changes from RFD to FLU
//                            joint_move_4.move_joint(joint3_pose + thetaLink4, WAIT);
//                            joint_move_3.move_joint(joint4_pose + phiLink3,WAIT);
                            ROS_WARN("stop end effector");
                            if( End_effector_enable == true ){
                                SetEndEffectorDirection(CLOCKWISE) ;
                                EndEffector(false);
                            }
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
//TODO:MR we changed the published data of link5 to Min length so no need to subtract ,wherever the cotton is we can directly feed the data(Z Value of Camera)
                            /*ROS_WARN( " R Value before adding joint5_pose : %f, ",rLink5) ;
                            ROS_WARN( "Current Postion of Link5 : %f, ",joint5_pose) ;
                            rLink5 += joint5_pose; 
                            ROS_WARN( " R Value after adding Joint5_Pose : %f, ",rLink5) ;*/


#if END_EFFECTOR_EN == true
#if AGGREGATE_PICK_EN == true
                            pre_start_delay = ((rLink5 - pre_start_len) / joint5_vel_limit);
                            //ROS_WARN("End-Effector will start in %f sec", pre_start_delay);
                            ROS_INFO("End-Effector will start in %f sec", pre_start_delay);
                            if (pre_start_delay <0)  //added by ribin for calling the pick cotton if the pre_start_delay is negative
                            {
                                pre_start_delay = 0;
                            }
                            /****** If directly accessing the GPIO the pre_start_delay timing *******/ 
                            //pick_cotton.command(pre_start_delay);
                            //gpioWrite(end_effector_on_pin,end_effector_on);                  // end_effector on
                            //gpioWrite(end_effector_direction_pin,end_effector_direction_cw); // end_effector_clockwise rotation

#endif
#endif
                            double link_5_start_time = currentTimeMillis();
                            // moving the joint5 forward
                            ROS_WARN( " R Value Passing to JOINT_MOVE rLink5_origin : %f, ",rLink5_origin) ;
                            joint_move_5.move_joint(rLink5_origin , WAIT);
                            VacuumPump("true") ; // uncomment this line if vacuum presuure is not good
                            float joint5_forward_delay; // = joint5_delay + picking_delay - pre_start_delay;

	                            //ROS_INFO("joint5_forward_delay %f ", joint5_forward_delay);     // joint5 forward delay
                            ROS_INFO("PRE START DELAY %f ", pre_start_len);     // joint5 forward delay
                            ros::Duration(pre_start_delay).sleep();
                            // end effector start after pre-start delay
                            if( End_effector_enable == true ){
                                VacuumPump("true") ;
                                SetEndEffectorDirection(CLOCKWISE) ;
                                EndEffector(true);
                            }
                            // Distance travelled NOW after moving time pre_start_delay
                            float DelayForRestOfDistanceForLink5  =  pre_start_len / joint5_vel_limit;
                            ros::Duration(DelayForRestOfDistanceForLink5).sleep();
                            double link_5_end_time = currentTimeMillis();
                            double link_5_moving_time = link_5_end_time - link_5_start_time;
                            ROS_INFO("TIMETO link_5_moving_time = %f ",link_5_moving_time);

                            ros::Duration(EERunTimeDuringL5ForwardMovement).sleep();

                            double end_picking_time =currentTimeMillis();
                            picked++;
                            double time_taken_for_each_pick = end_picking_time - picking_time_started;
                            ROS_INFO("TIMETO pick one cotton by arm and EEffector = %f ",time_taken_for_each_pick);

                            // moving joint5 backward
                            //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                            joint_move_5.move_joint(joint5_pose, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                           ROS_WARN("\nJOINT5 POSE VALUE :%f\n",joint5_pose); 
			   // end effector stop 
                            ros::Duration(EERunTimeDuringL5BackwardMovement).sleep();
	
			    //Droping the Cotton From End Effector after arm Collapsed
                            //ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();//Waiting for the arm to be collapsed
		            if (EndEffectorDropConveyor) {
                            EndEffectorDrop(DROP_EEF);
                            }
                            //ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();
                            // Reverse The End Effector 
                            if( End_effector_enable == true ){
                                SetEndEffectorDirection(ANTICLOCKWISE) ; //reverse stopped 29jul2023
                               // SetEndEffectorDirection(CLOCKWISE) ;
                                EndEffector(true); // 19-OCT-2022
                                VacuumPump("true") ;
                                ROS_WARN("Opening Shutter \n");
                                cotton_drop_solenoid_shutter();
                            }
                            // Run the EndEffector for EERunDuringReverseRotation

                            ros::Duration(EERunTimeDuringReverseRotation ).sleep();
		            if (EndEffectorDropConveyor) {
                            EndEffectorDrop(STOP_EEF);
			    }
                            //Stop end effector after This
                            if( End_effector_enable == true ){
                                SetEndEffectorDirection(CLOCKWISE) ;    // 19-OCT-2022
                                EndEffector(false);
                                //VacuumPump("false") ; // Enable this statement if vacuum is continously required
                            }

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

                            //drop_cotton.command(joint5_delay);
                            //gpioWrite(end_effector_on_pin,end_effector_on);                  // end_effector on
                            //gpioWrite(end_effector_direction_pin,end_effector_direction_acw); // end_effector_anti_clockwise rotation
                            //SetEndEffectorDirection(false) ;
                            //gpioWrite(end_effector_on_pin,end_effector_off);                  // end_effector off
                            //ros::Duration(2).sleep();
                            //EndEffector(false) ;


                        }
                    else
                    {
                        ROS_ERROR("Cotton target %d is out of bound", trgt);
                    }
                    writeFormattedMessageToLogFile(log_movement," RLINK5_ORIGIN : %f, THETALINK4 : %f, PHILINK3 : %f",
                            rLink5_origin, thetaLink4, phiLink3);
                    ros::spinOnce();
                    if((joint_move_3.error_code != NO_ERROR) ||
                            (joint_move_4.error_code != NO_ERROR) ||
                            (joint_move_5.error_code != NO_ERROR))
                    {
                        ROS_ERROR("PROBLEM IN SOME JOINT");
                        problem_indicator_out.command(ERROR);
                        break;
                    } // if ((joint_move_3.error ...

                    // Move Only Link5 ie Z arm to it retracted position
                    // We will keep the X and Y joints at the same position and move to the next target locatio
                    // This is to minimize time on X and Y joint movement during retraction
                    // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                    //joint_move_5.move_joint(joint5_homing_position, WAIT);   
                    //ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
                    //ros::Duration(2).sleep();
                    //joint_move_3.move_joint(joint3_pose,WAIT);
                    //ros::Duration(2).sleep();
                    //joint_move_4.move_joint(joint4_pose, WAIT);

                }
                end_time = currentTimeMillis();
                double time_taken_for_picking_n_number_of_cotton = end_time - start_time_to_pick_n_number_of_cotton;
                ROS_INFO("TIMETO pick %d number of cotton = %f",picked,time_taken_for_picking_n_number_of_cotton);
                //ROS_INFO("TIMETO pick %d number of cotton %f",picked,(int)time_taken_for_picking_n_number_of_cotton);
                ROS_INFO("NUMBEROF cotton picked = %d",picked);

                // turn off vaccum motor 
                if(Global_vaccum_motor == true){
                    VacuumPump(false) ;
                }
                if( End_effector_enable == true ){
                    SetEndEffectorDirection(CLOCKWISE) ; //19-OCT-2022
                    EndEffector(false);
                }

                if (picked > 4){

                    //joint_move_5.move_joint(joint5_homing_position, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                    joint_move_5.move_joint(joint5_pose, WAIT);   // Move joint5 fully back TODO change 0.001 to joint_zero_poses[joint3_cnt]
                    ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();
                    //ros::Duration(3).sleep();
                    joint_move_3.move_joint(joint3_parking_pose,WAIT);
                    ROS_WARN("JOINT3_PARKING_POSE : %f ",joint3_parking_pose); //add by gokul
//ros::Duration(1).sleep();
                    joint_move_4.move_joint(joint4_parking_pose,WAIT);
                    //ros::Duration(1).sleep();
                    //ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();

                    //lid_open.command(0);
                    //cotton_drop_shutter();
	            ROS_WARN("Opening Shutter \n");
                    ros::Duration(MIN_SLEEP_TIME_FOR_COTTON_DROP).sleep();
		   // cotton_drop_solenoid_shutter();
                    // Activate Transport unit to continue picking.
                    // TransportUnitActivate() ;

                    //ros::Duration(6).sleep(); //TODO:Change this dealy to Cotton Transport Variable
                    picked = 0;

                }

                        VacuumPump(false);

                // dropping all the cottons at end of every pose
            } // if ( NoOfTargetLocations > 0 ) 

        }
        /* Move joint4 to align with the row, so that height can be changed */
        joint_move_5.move_joint(joint5_homing_position, WAIT);//TODO This homing Position is different from Initialisation homing position but both can be same
        ROS_WARN(" joint5_homing_position :%f",joint5_homing_position);
	//ros::Duration(2).sleep();
        joint_move_4.move_joint(joint4_homing_position, WAIT); // TODO move it to joint4_homing_position
        joint_move_3.move_joint(joint3_homing_position, WAIT); //add by gokul
        ROS_WARN("JOINT3_HOMING_POSITION :%f ",joint3_homing_position);
        ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
        //ros::Duration(0.200).sleep();
        ROS_INFO("all joints are in homing_position going for l2 movement");
        //joint_move_2.move_joint(height_scan_value,WAIT);


        if(height_scan_enable ==true)
        {
            ROS_INFO("height_scan is enabled, height_scan_value:= %f", height_scan_value);
            ROS_INFO("moving to top position");
            joint_move_2.move_joint(height_scan_min,WAIT);
            //lid_open.command(1);
            //ros::Duration(5).sleep();
            ros::Duration(l2_homing_sleep_time).sleep();
            srv_idle.request.joint_id = 3; //we are calling the idle for l2 here after moving to a posiiton
            ros::Duration(l2_idle_sleep_time).sleep();
            if(joint_move::joint_idle_service.call(srv_idle)!=true)
            {
                ROS_ERROR("Joint2 idle, Reason: %s", srv.response.reason.c_str());
                problem_indicator_out.command(ERROR);
                return 0;
            } //TODO:Joint2_Idle_Removing_To_Stop_Moving


            end_time = currentTimeMillis(); 
            double time_taken = end_time - start_time; // converted into milli seconds 
            ROS_INFO("TIMETO taken by height scan is = %f ", time_taken); 
            print_timestamp(" height scan done ");
        }
        //this Command is for to activate Transport unit
        //transport_shutter();
        //CallOdriveHWLoop();
        /*
        ros::Duration elapsed_time(0) ;
        odrive_hw_interface->read(elapsed_time );
        odrive_hw_interface->PublishJointStateValues() ;//ODriveJointStatePublisher();
        odrive_hw_interface->write(elapsed_time );
        // Print the argument, static and global variables
        //ROS_INFO("In ODriveJointStatePublisher loop with threadID : %d", *myid) ;
        */
        last_time =currentTimeMillis();
        loop_rate.sleep() ;
    }
  

    ////#####################l2 height_scan_step ##########################//
    //#endif
    ROS_INFO("%s putting the arm in Parking Position\n", __func__);
    /* Parking Position */
    {
    joint_move_5.move_joint(joint5_parking_pose, WAIT);
    ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
    joint_move_4.move_joint(joint4_parking_pose, WAIT);
    joint_move_3.move_joint(joint3_parking_pose, WAIT);
    ros::Duration(MIN_SLEEP_TIME_FORMOTOR_MOTION ).sleep();
    }
    writeFormattedMessageToLogFile(log_filename,"\n ");
    writeFormattedMessageToLogFile(log_movement,"\n ");
    CallOdriveHWLoop();
    //
    return 0;
}
