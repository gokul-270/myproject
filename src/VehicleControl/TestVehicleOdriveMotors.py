#!/usr/bin/python3
#Created by: Rajamanickam
#History :11jul2023
# This program will helps you to check individual motor and moves to 0.3 position and reads the postion same as the the distance moved and then its rebooted
#change the Motor_ID to check other motors based on the motor id you entered





import VehicleCanBusInterface
import OdriveCanBusHelperFunctions
import time

#CanIds of the motor
'''
LeftDriveMotor            0 
LeftSteerMotor            3
MiddleDriveMotor          4
MiddleSteerMotor          5
RightDriveMotor           2
RightSteerMotor           1
'''

Motor_ID=4 # This is the CanID of the motor being tested
Distance = 0.2

def Basic_Tests():
   VehicleCanBusInterface.Initialise()
   time.sleep(5)
   print("Moving Motor by a rotation of :  ", Distance) 
   VehicleCanBusInterface.SetMotorToClosedLoop(Motor_ID)
   print("putting closed  loop")
   VehicleCanBusInterface.MoveMotorPositionAbsolute(Motor_ID,Distance)
   time.sleep(5)

   print("Moving Motor by a rotation of :  ", -1.0 * Distance) 
   VehicleCanBusInterface.MoveMotorPositionAbsolute(Motor_ID, -1.0 * Distance)
   time.sleep(5)
   OdriveCanBusHelperFunctions.GetPositionAndVelocity(VehicleCanBusInterface.bus,Motor_ID)
   VehicleCanBusInterface.RebootOdrive(Motor_ID)
   

if __name__== "__main__" :
   print(" This program will have to be run in the directory /home/ubuntu/pragati/src/VehicleControl")
   print(" To test Different Motor, change the MotorI_ID values to the desired Motor CanIDs")
   Motor_ID = int(input("Motor_ID"))
   Basic_Tests()
