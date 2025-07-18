
/*  Author: Saurabh Bansal
    Desc:   O-Drive Serial Interface, to be used to communicate with O-Drive Board
*/

#include <odrive_control/odrive_serial_interface.h>


/**
 * Construct ODriveSerial object linked to the given serial port.
 * @param serial Serial port to use to communicate to the ODrive
 */
ODriveSerial::ODriveSerial(comm_serial& serial, int motor_number, long limit_min, long limit_max)
    : serial_(serial), motor_number_(motor_number), limit_min_(limit_min), limit_max_(limit_max)
{
}

void ODriveSerial::Get_SerialNumber(char* serial_number)
{
}

bool ODriveSerial::Run_State(int requested_state, bool wait)
{
}

void ODriveSerial::Save_Configuration(void)
{
}

void ODriveSerial::Erase_Configuration(void)
{
}

void ODriveSerial::Reboot(void)
{
}
void ODriveSerial::Trajectory_write(std::string property, double value)

  {
    property +='(';
    property += std::to_string(values);
    property +=')';
    std::cout<<"property :"<<property<<std::endl;
    SendCommand(property, PR_WRITE, AXIS_COMMAND);

  }


void ODriveSerial::WriteProperty(std::string property)
{
    SendCommand(property, PR_WRITE);
}

void ODriveSerial::WriteProperty(std::string property, bool value)
{
    property += ' ';
    property += std::to_string(value);
    SendCommand(property, PR_WRITE);
}

void ODriveSerial::WriteProperty(std::string property, int value)
{
    property += ' ';
    property += std::to_string(value);
    SendCommand(property, PR_WRITE);
}

void ODriveSerial::WriteProperty(std::string property, float value)
{
    property += ' ';
    property += std::to_string(value);
    SendCommand(property, PR_WRITE);
}

void ODriveSerial::ReadProperty(std::string property, float* value)
{
    std::string response;
    SendCommand(property, PR_READ);
    response = serial_.comm_read();
    *value = std::stof(response);
}

void ODriveSerial::ReadProperty(std::string property, int* value)
{
    std::string response;
    SendCommand(property, PR_READ);
    response = serial_.comm_read();
    *value = std::stof(response);
}

void ODriveSerial::SendCommand(std::string property, bool read_write)
{
    std::string command;
    if(read_write == PR_WRITE)
        command.append("w axis");
    else if(read_write == PR_READ)
        command.append("r axis");
    command += std::to_string(motor_number_);
    command += '.';
    command.append(property);
    command += '\n';

    serial_.comm_write(command);
}
