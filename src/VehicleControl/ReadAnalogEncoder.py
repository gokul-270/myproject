#!/usr/bin/python3
# HISTORY : Last updated 28MAY2023
# Read IMU (from Arduino 9Axis Motion shield interfaced to ARDUINO using USB port connected to RPI )

#from ast import For
import time
#from tkinter import W

# Converts Current angle in degrees to NoOfMotorRotations , a floating point value
# This considers the gear ration in use
# ASSUMPTIONS : At power up it is assumed that the wheel is at zero yaw angle
# TODO : How to correct situations where wheel is not aligned to zero yaw angle at Powerup4.

 
def ConvertEncoderValueToMotorAngle( current_Angle ):
    Maximumum_EncoderValue = 4096
    # The min_steer_limit is derived from a 5 cm error from the straigth line  
    min_steer_limit = 3  # in Degrees 
    steering_gear_ratio = float(50) # For steering Motor Gear is 1:50
    steeringGearRatioinRotation = float( steering_gear_ratio / 360)  # rotations
    req_SteeringAngle = int(0)
    CenterPointValue=1296
    AnglePerEncoderRotation=360/Maximumum_EncoderValue
    Current_EncoderValue = ReadEncoder()
    if (abs( current_EncoderValue - CenterPointValue)>10):
       if(Current_EncoderValue > CenterPointValue):
          Current_correctionAngle = AngleBasedOnRotation*(current_EncoderValue-CenterPointValue)
       if(Current_EncoderValue < CenterPointValue):
          Current_correctionAngle = AngleBasedOnRotation*(CenterPointValue-current_EncoderValue)

    MotorRotation = Current_correctionAngle * steeringGearRatioinRotation 
    return(MotorRotation)


import serial
#def InitializeIMU():
ser = serial.Serial('/dev/ttyUSB0' , 9600, timeout = 1)
ser.reset_input_buffer()

#define a Variable IMUValueInFloat so that it can store the last readValue
global EncoderValueInFloat
EncoderValueInFloat = 0
# ReadIMU Reads value through the serial port of Arduino 
#    IMU is mounted on Arduino and the values are pushed to the serial port in the Arduino in asynchronous manner
#    TODO will have to change this to read directly to RPI4 to eliminate the Arduino interface.
def ReadEncoder():
    #import time
    global ser
    global EncoderValueInFloat
    #ser.reset_input_buffer()
    line =""
    #Sampling time of IMU is 0.02 seconds. Reading faster than that will not give reliable numbers
    #time.sleep(0.02)
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').rstrip()
            EncoderValueInFloat = float(line)
            #print("ReadIMU() value read")
            #print(line)
        except: 
            print("ser.readline().decode() error")
        # Ignore the first value of the IMU and read again
        # There are extraneous characters from Serial port which we are trying to ignore
        #  TODO : replace the serial port to direct read from RPI4
       # try:
        #    IMUValueInFloat = float(line)
        #except:
         #   print("Exception, Error in string to float conversion")
          #  print(line)
        #ser.reset_input_buffer()
    else: print("ReadIMU() :: No Value Read")
    print("ReadEncoder Value Read: {}".format(EncoderValueInFloat))
    return(EncoderValueInFloat)
    #return(0)
        


if __name__ == "__main__":
    Initialise()
    while True:
        print ("Reading EncoderValue")
        CurrentYawAngle = ReadEncoder()
        #MotorRotation = ConvertYawToMotorRotation(CurrentYawAngle)
        print("Current Value : {}".format(CurrentYawAngle))
        time.sleep(0.02)
#End of __main__ 
