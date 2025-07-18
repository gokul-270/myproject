#!/usr/bin/python3
#History :11jul2023
#Created by: Rajamanickam
# This program will helps you to check individual motor and moves to 0.3 position and reads the postion same as the the distance moved and then its rebooted
#change the Motor_ID to check other motors based on the motor id you entered
# fixed the motor id to 0, 1, 2




import VehicleCanBusInterface
import OdriveCanBusHelperFunctions
import time

Motor_ID=0
Distance =0.3

def Basic_Tests(Motor_ID):
   print("Start of  Test for the motor with CANNode ID : ", Motor_ID)
   time.sleep(5)

   VehicleCanBusInterface.Initialise()
   time.sleep(5)
   print("Reading the Motor Position")
   #OdriveCanBusHelperFunctions.GetPositionAndVelocity(VehicleCanBusInterface.bus,Motor_ID)
   print(" The Motor should move for index Search")
   VehicleCanBusInterface.SetMotorToEncoderIndexSearch(Motor_ID)
   time.sleep(5)

   print("putting To  closed  loop")
   VehicleCanBusInterface.SetMotorToClosedLoop(Motor_ID)
   time.sleep(1)
   print(" Please Check Whether Motor is in Closed Loop ie held in position ") 
   time.sleep(5)
   
   print("putting Motor To Idle ")
   VehicleCanBusInterface.SetMotorToIdle(Motor_ID)
   time.sleep(1)
   print(" Please Check Whether Motor is in Idle ie freely moving  ") 
   time.sleep(5)


   print("putting To  closed  loop")
   VehicleCanBusInterface.SetMotorToClosedLoop(Motor_ID)
   time.sleep(1)
   print(" Please Check Whether Motor is in Closed Loop ie held in position ") 
   time.sleep(5)

   print("Moving Motor by Rotation = ", Distance)
   print("See whether the Motor is  Rotation x= ")
   VehicleCanBusInterface.MoveMotorPositionAbsolute(Motor_ID,Distance)
   time.sleep(1)

   print("Reading the Motor Position")
   #OdriveCanBusHelperFunctions.GetPositionAndVelocity(VehicleCanBusInterface.bus,Motor_ID)
   VehicleCanBusInterface.RebootOdrive(Motor_ID)

   print(" Completed Testing The motor with CANNode ID : ", Motor_ID)
   print(" THe motor should now be in idle , ie freely moving") 


if __name__== "__main__" :
   Basic_Tests(0)
   Basic_Tests(1)
   Basic_Tests(2)
