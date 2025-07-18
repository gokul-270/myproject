/**
*  Source(Saurabh): https://github.com/UDOOboard/serial_libraries_examples.git
*  Added: tcflush to flush any data from Terminal IO buffers
*
*  Copyright (C) 2014 Ekironji <ekironjisolutions@gmail.com>
*
*  This file is part of serial libraries examples for UDOO
*
*  Serial libraries examples for UDOO is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This libraries are distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#ifndef ODRIVE_CONTROL__HARDWARE_SERIAL_INTERFACE_H
#define ODRIVE_CONTROL__HARDWARE_SERIAL_INTERFACE_H

#include <termios.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <libudev.h>
#include <sys/stat.h>

#define SERIAL_BUFFER_SIZE      128

class comm_serial
{
    int fd;
    char buffer[SERIAL_BUFFER_SIZE];

    int set_interface_attribs(int speed);
    void set_blocking(int should_block);
    void ReadUsbIdentifiers(std::string dev_path, std::string& serial_number);
public:
    /* Send Command to Serial Port */
    bool comm_write(std::string command)
    {
        tcflush(fd, TCIFLUSH);      // To flush the terminal I/O buffers
        int n = write(fd, command.c_str(), command.length());
        if(n != command.length())
            return false;
        return true;
    }
    /* Read Response from Serial Port */
    std::string comm_read(void)
    {
        std::string response;
        memset(buffer, 0, SERIAL_BUFFER_SIZE);
        int n = read(fd, buffer, SERIAL_BUFFER_SIZE);
        if(n > 0)
        {
            response = buffer;
        }
        return response;
    }
    /* CONSTRUCTOR */
    comm_serial(std::string serial_number, int speed)
    {
        std::string portname;
        std::string rec_serial_number;
        std::cout << "Searching for " << serial_number << std::endl;
        for(int i = 0; i < 3; i++)
        {
            portname = "/dev/ttyACM" + std::to_string(i);
            ReadUsbIdentifiers(portname, rec_serial_number);
            if(rec_serial_number == serial_number)
            {
                std::cout << "SUCCESS: Found serial number " << serial_number << " on device " << portname << std::endl;
                break;
            }
        }
        fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if(fd < 0)
        {
            printf("error %d opening %s", errno, portname.c_str());
        }
        set_interface_attribs(speed);
        set_blocking(1);                            // set blocking
    }
};

int comm_serial::set_interface_attribs(int speed)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0)
    {
        printf("error from tcgetattr: %d", errno);
        return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
    tty.c_iflag &= ~IGNBRK;                         // ignore break signal
    tty.c_lflag = 0;                                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void comm_serial::set_blocking(int should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0)
    {
        printf("error %d from tggetattr", errno);
        return;
    }
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout
    if(tcsetattr(fd, TCSANOW, &tty) != 0)
        printf("error %d setting term attributes", errno);
}

/* Source : https://stackoverflow.com/questions/49207803/how-to-get-the-usb-vid-pid-and-serial-number-of-a-device-in-ubuntu-using-c-fr?rq=1
*/
void comm_serial::ReadUsbIdentifiers(std::string dev_path, std::string& serial_number)
{
    auto udev = udev_new();
    if(!udev)
        return;
    
    struct stat statbuf;
    if(stat(dev_path.c_str(), &statbuf) < 0)
        return;
    auto type = S_ISBLK(statbuf.st_mode) ? 'b' : S_ISCHR(statbuf.st_mode) ? 'c' : 0;
    
    auto opened_dev = udev_device_new_from_devnum(udev, type, statbuf.st_rdev);
    auto dev = opened_dev;
    
    while(dev != nullptr)
    {
        auto serial = udev_device_get_sysattr_value(dev, "serial");
        if(nullptr == serial)
        {
            dev = udev_device_get_parent(dev);
        }
        else
        {
            std::cout << "VID: " <<  udev_device_get_sysattr_value(dev, "idVendor") << std::endl;
            std::cout << "PID: " <<  udev_device_get_sysattr_value(dev, "idProduct") << std::endl;
            std::cout << "Serial Number: " <<  serial << std::endl;
            serial_number = serial;
            break;
        }
    }
    if(opened_dev)
        udev_device_unref(opened_dev);
    udev_unref(udev);
}

#endif  // ODRIVE_CONTROL__HARDWARE_SERIAL_INTERFACE_H
