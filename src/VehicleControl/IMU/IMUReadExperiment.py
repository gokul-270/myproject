#!/usr/bin/python3
# HISTORY : Last updated 28MAY2023
# Read IMU (from Arduino 9Axis Motion shield interfaced to ARDUINO using USB port connected to RPI )

#from ast import For
import time
#from tkinter import W

# Converts Yaw angle in degrees to NoOfMotorRotations , a floating point value
# This considers the gear ration in use
# ASSUMPTIONS : At power up it is assumed that the wheel is at zero yaw angle
# TODO : How to correct situations where wheel is not aligned to zero yaw angle at Powerup 
def ConvertYawToMotorRotation( current_yaw ):
    max_steer_limit = 45 # in Degreees
    neg_max_steer_limit = -45  # in Degrees
    # The min_steer_limit is derived from a 5 cm error from the straigth line  
    min_steer_limit = 3  # in Degrees 
    steering_gear_ratio = float(50) # For steering Motor Gear is 1:50
    steeringGearRatioinRotation = float( steering_gear_ratio / 360)  # rotations
    req_yaw = int(0)
    if(current_yaw>180):
        current_yaw = current_yaw - 360
    correction_angle = (req_yaw - current_yaw)
    if(abs(correction_angle) <= min_steer_limit):
        correction_angle = 0
    if(abs(correction_angle) > max_steer_limit):
        if(correction_angle > 0):
            correction_angle = max_steer_limit * steeringGearRatioinRotation 
        elif(correction_angle <0):
            correction_angle = neg_max_steer_limit * steeringGearRatioinRotation 
    MotorRotation = correction_angle * steeringGearRatioinRotation 
    return(MotorRotation)


import serial
#def InitializeIMU():
ser = serial.Serial('/dev/ttyACM0' , 9600, timeout = 1)
ser.reset_input_buffer()

#define a Variable IMUValueInFloat so that it can store the last readValue
global IMUValueInFloat
IMUValueInFloat = 0
# ReadIMU Reads value through the serial port of Arduino 
#    IMU is mounted on Arduino and the values are pushed to the serial port in the Arduino in asynchronous manner
#    TODO will have to change this to read directly to RPI4 to eliminate the Arduino interface.
def ReadIMU():
    #import time
    global ser
    global IMUValueInFloat
    #ser.reset_input_buffer()
    line =""
    #Sampling time of IMU is 0.02 seconds. Reading faster than that will not give reliable numbers
    time.sleep(0.02)
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').rstrip()
            IMUValueInFloat = float(line)
            #print("ReadIMU() value read")
            #print(line)
        except: 
            print("ser.readline().decode() error")
        # Ignore the first value of the IMU and read again
        # There are extraneous characters from Serial port which we are trying to ignore
        #  TODO : replace the serial port to direct read from RPI4
        try:
            IMUValueInFloat = float(line)
        except:
            print("Exception, Error in string to float conversion")
            print(line)
        #ser.reset_input_buffer()
    else: print("ReadIMU() :: No Value Read")
    print("ReadIMU Value Read: {}".format(IMUValueInFloat))
    return(IMUValueInFloat)
    #return(0)
        
def SteeringVehicle():
    #ReadIMU()
    #SteeringMotors(Odriveid2, Motorid0, Motorid1, MotorRotation)
    #time.sleep(1)
    print("VEHICLE_STEERING")


def RunVehicle():
    print("VEHICLE_STEERING")

def Initialise():
    ser.reset_input_buffer()
#END of def Initialise():

def Execute():
    while True:
        ser.reset_input_buffer()
        print ("Reading IMU")
        CurrentYawAngle = ReadIMU()
        MotorRotation = str(ConvertYawToMotorRotation(CurrentYawAngle))
        print("Current Value : {}".format(CurrentYawAngle))
        print("MotorRoation : {}".format(MotorRotation))
        #Idle.has_been_called = False
        #RunVehicle()
        #MoveWheels()
        #print("Finished Moving Vehicle ")
        #SteeringIteration = 0
        #SteeringVehicle()
        print("Vehicle steering")
        #while True:
          #print("Steering Iteration {}".format(SteeringIteration))
          #SteeringIteration += 1
          #CurrentYawAngle = ReadIMU()
          #MotorRotation = str(ConvertYawToMotorRotation(CurrentYawAngle))
          #print("Current Value : {}".format(CurrentYawAngle))
          #print("MotorRoation : {}".format(MotorRotation))
          #SteeringVehicle()
          #print( " axis0 Trajectory Status : {}".format( myDrive.axis0.controller.trajectory_done ))
          #print( " axis1 Trajectory Status : {}".format( myDrive.axis1.controller.trajectory_done ))
          #VehicleStopped = False     
          #VehicleStopped = myDrive.axis0controller.trajectory_done and myDrive.axis1.controller.trajectory_done
          #if (VehicleStopped): 
            #print ("Vehicle_stopped")
            #break
#end of def Execute()

if __name__ == "__main__":
    Initialise()
    while True:
        #ReadIMU()
        print ("Reading IMU")
        CurrentYawAngle = ReadIMU()
        MotorRotation = ConvertYawToMotorRotation(CurrentYawAngle)
        print("Current Value : {}".format(CurrentYawAngle))
        print("MotorRoation : {}".format(MotorRotation))
        #print( "Steering Angle ")
        #print(MotorRotation)
        #print("CurrentYawAngle" )
        #print(CurrentYawAngle )
        time.sleep(0.02)
#End of __main__ 
