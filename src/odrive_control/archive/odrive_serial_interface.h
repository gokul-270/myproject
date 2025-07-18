
/*  Author: Saurabh Bansal
    Desc:   O-Drive Hardware control, to be used with ros-control
*/

#ifndef ODRIVE_CONTROL__ODRIVE_SERIAL_INTERFACE_H
#define ODRIVE_CONTROL__ODRIVE_SERIAL_INTERFACE_H

#include <odrive_control/hardware_serial_interface.h>
#include <unistd.h>

/* Definitions */
#define START_BYTE              1
#define NL_LEN                  0
#define PR_WRITE                true
#define PR_READ                 false
#define AXIS_COMMAND            true
#define SYS_COMMAND             false
#define WAIT_TO_NEXT_COMMAND    500             // in micro-seconds
#define STATE_RQST_TIMEOUT      (10*1000000)    // 10 seconds, in micro-seconds
#define STATE_RQST_COUNTS       (STATE_RQST_TIMEOUT / WAIT_TO_NEXT_COMMAND)
#define STATE_RQST_WAIT         true
#define STATE_RQST_NO_WAIT      false

/* Properties that can be Read / Written to O-Drive */
#define ODRV_SERIAL_NUMBER          "serial_number"
#define ODRV_CURRENT_LIM_RW         "motor.config.current_lim"
#define ODRV_ENCODER_IS_READY_RW    "encoder.is_ready"
#define ODRV_MEAS_CURRENT_R         "motor.current_control.Iq_measured"
#define ODRV_POSITION_RW            "controller.pos_setpoint"
/* this is for trajectory_control*/
#define ODRV_TRAJ_RW                "controller.move_to_pos"
/*we have to test this function extensively because this is a funtion call where we have to pass the float value, other commands that we were using
is just a value that we are assigning" */

#define ODRV_MOVE_TO_POSE           "t "   //move to pose function 
#define ODRV_POS_ERROR_R            "controller.error"
#define ODRV_VELOCITY_RW            "controller.config.vel_limit"
#define ODRV_CURRENT_VELOCITY_R     "encoder.vel_estimate"
#define ODRV_ENCODER_CPR_RW         "encoder.config.cpr"
#define ODRV_SPIN_MOTOR
#define ODRV_CURRENT_POSITION_R     "encoder.shadow_count"
#define ODRV_POS_ESTIMATE           "encoder.pos_estimate"
#define ODRV_POS_GAIN_RW            "controller.config.pos_gain"
#define ODRV_VEL_GAIN_RW            "controller.config.vel_gain"
#define ODRV_VEL_INTEGRATOR_GAIN_RW "controller.config.vel_integrator_gain"
#define ODRV_DRIVE_TEMP_R           "motor.get_inverter_temp()"
#define ODRV_MOTOR_TEMP_R           ""
#define ODRV_DRIVE_FAULT_R          "motor.gate_driver.drv_fault"
#define ODRV_INDEX_FOUND_R          "encoder.index_found"
#define ODRV_AXIS_ERROR_R           "error"
#define ODRV_SAVE_CONFIG            "ss"
#define ODRV_ERASE_CONFIG           "se"
#define ODRV_SYSTEM_REBOOT          "sr"
#define ODRV_REQUEST_STATE          "requested_state"
#define ODRV_CURRENT_STATE          "current_state"
//added by ribin for reversing index search
#define ODRV_INDEX_SEARCH_REVERSE_VEL            "config.lockin.vel"
#define ODRV_INDEX_SEARCH_REVERSE_ACCEL         "config.lockin.accel"
#define ODRV_INDEX_SEARCH_REVERSE_RAMP_DISTANCE  "config.lockin.ramp_distance"
//done

#define AXIS_STATE_UNDEFINED                    0   //<! will fall through to idle
#define AXIS_STATE_IDLE                         1   //<! disable PWM and do nothing
#define AXIS_STATE_STARTUP_SEQUENCE             2   //<! the actual sequence is defined by the config.startup_... flags
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE    3   //<! run all calibration procedures, then idle
#define AXIS_STATE_MOTOR_CALIBRATION            4   //<! run motor calibration
#define AXIS_STATE_SENSORLESS_CONTROL           5   //<! run sensorless control
#define AXIS_STATE_ENCODER_INDEX_SEARCH         6   //<! run encoder index search
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION   7   //<! run encoder offset calibration
#define AXIS_STATE_CLOSED_LOOP_CONTROL          8   //<! run closed loop control

/****  Added this for debug purposes for backtracking the stack when there is an error */

#include <stdio.h>
#include <execinfo.h>
#include <stdlib.h>


void handler(char *caller) {
  void *array[10];
  size_t size;
  printf("Stack Trace Start for %s\n",caller);
  size = backtrace(array, 10);
  backtrace_symbols_fd(array, size, 2);
  printf("Stack Trace End\n");
}
/*
void car() {
    handler("car()");
    printf("Continue Execution");
}
*/

class ODriveSerial
{
public:
    // Constructor
    ODriveSerial(comm_serial& serial, int motor_number)
        : serial_(serial), motor_number_(motor_number)
    {
    }

    /*This funtion  Trajectory_write is added by ribin for controlling the joint  using the trajectory control, the reeason
    //for writing this function is because the trajectory is a funtion in odrive */
    void Trajectory_write(std::string property, double value)
    {
      property +='(';
      property += std::to_string(value);
      property +=')';
      std::cout<<"trajectory property :" <<property<<std::endl;
      SendCommand(property, PR_WRITE, AXIS_COMMAND);

    }

    void WriteProperty(std::string property)
    {
        SendCommand(property, PR_WRITE, AXIS_COMMAND);
    }
    void WriteProperty(std::string property, double value)
    {
        property += ' ';
        property += std::to_string(value);
        SendCommand(property, PR_WRITE, AXIS_COMMAND);
	std::cout<<"sending position doulbe "<< property<<std::endl;

    }

    void WriteProperty(std::string property, long value)
    {
        property += ' ';
        property += std::to_string(value);
        SendCommand(property, PR_WRITE, AXIS_COMMAND);
	std::cout<<"sending position long "<< property<<std::endl;

    }
    void WriteProperty(std::string property, bool value)
    {
        property += ' ';
        property += std::to_string(value);
        SendCommand(property, PR_WRITE, AXIS_COMMAND);
        std::cout<<"sending position bool "<< property<<std::endl;

    }
    void WriteProperty(std::string property, float value)
    {
        property += ' ';
        property += std::to_string(value);
        SendCommand(property, PR_WRITE, AXIS_COMMAND);
        std::cout<<"sending position bool "<< property<<std::endl;

    }
   
   void Write_command(std::string property,int motor_id, long value)
        {
        property += std::to_string(motor_id);
	property +=' ';
        property += std::to_string(value);       
        property +='\n';
	serial_.comm_write(property);
	std::cout<<"sending position command "<< property<<std::endl;
        }
   void Write_command_float(std::string property,int motor_id, float value)
        {
        property += std::to_string(motor_id);
	property +=' ';
        property += std::to_string(value);       
        property +='\n';
	serial_.comm_write(property);
	std::cout<<"sending position command "<< property<<std::endl;
        }
    void ReadProperty(std::string property, double* value)
    {
        std::string response = "";
        SendCommand(property, PR_READ, AXIS_COMMAND);
        response = serial_.comm_read();
      //  std::cout<<"response from ReadProperty double =: "<< response <<std::endl;
        try {
          *value = std::stod(response);
          }
          catch(const std::invalid_argument)
        { std::cout<<"error in stod"<<std::endl;}
    }
    void ReadProperty(std::string property, long* value)
    {
        std::string response = "";
        SendCommand(property, PR_READ, AXIS_COMMAND);
        response = serial_.comm_read();
      //  std::cout<<"response from ReadProperty long =: "<< response <<std::endl;
        try {
          *value = std::stol(response);
          }
          catch(const std::invalid_argument)
        {
          handler((char *) "ReadProperty");
          //ROS_INFO("ReadPropery long string %s",  response);
          ROS_INFO("ReadPropery long value %ld",  *value);
          std::cout<<"error in stol"<<std::endl;
        }
      }



    long GetError(void)
    {
        long response = 0;
        ReadProperty(ODRV_AXIS_ERROR_R, &response);
        return response;
    }
    void Save_Configuration(void)
    {
        std::string command = ODRV_SAVE_CONFIG;
        serial_.comm_write(command);
    }
    void Erase_Configuration(void)
    {
        std::string command = ODRV_ERASE_CONFIG;
        serial_.comm_write(command);
        wait_to_idle();
    }
    bool Reboot(void)
    {
        std::string command = ODRV_SYSTEM_REBOOT;
        serial_.comm_write(command);
        return wait_to_idle();
    }
    // Enter in some state
    bool Run_State(long requested_state, bool wait_for_idle)
    {
        WriteProperty(ODRV_REQUEST_STATE, requested_state);

        // Check whether system has entered the requested_state
        long current_state = AXIS_STATE_UNDEFINED;
        ReadProperty(ODRV_CURRENT_STATE, &current_state);
        if(current_state != requested_state)
            return false;

        // If we need to wait for AXIS_STATE_IDLE
        if(wait_for_idle == STATE_RQST_WAIT)
            return wait_to_idle();

        return true;
    }
private:
    // Send the command to Read / Write some property
    void SendCommand(std::string property, bool read_write, bool axis)
    {
        std::string command;
        if(read_write == PR_WRITE)
            command.append("w ");
        else if(read_write == PR_READ)
            command.append("r ");
        if(axis == AXIS_COMMAND)
        {
            command.append("axis");
            command += std::to_string(motor_number_);
            command += '.';
        }
        command.append(property);
        command += '\n';
        serial_.comm_write(command);
        usleep(WAIT_TO_NEXT_COMMAND);
        //std::cout << "Sending command: " << command << std::endl;
    }
    // Wait till the current_state is changed to AXIS_STATE_IDLE
    bool wait_to_idle(void)
    {
        long t = 0;
        long state = 0;
        for(t = 0; t < STATE_RQST_COUNTS; t++)
        {
            ReadProperty(ODRV_CURRENT_STATE, &state);
            if(state == AXIS_STATE_IDLE)
            {
                // If State becomes IDLE, return true i.e. success
                return true;
            }
        }
        return false;
    }
    comm_serial& serial_;
    int motor_number_;
};

#endif // ODRIVE_CONTROL__ODRIVE_SERIAL_INTERFACE_H
