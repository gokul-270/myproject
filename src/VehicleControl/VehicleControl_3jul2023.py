# History
# Last Date When Code Updated : 9JUN2023

# 23 November 2022 :
#   1. Added checks to move Vehicle only if the Value is larger than MinimumVehicleDistanceResolution
#   2. Steer Angle only if it is more than MinimumVehicleSteeringAngle 
#   3. Limited the Maximum steering Angle to MaximumSteeringAngle  
#   4. Added Code to Read VehicleMode, VehicleDirection, VehicleStop, ArmStart, ArmShutdown
# 25 November 2022:
#   1. Added output Pins control and simple testcode TestOutputPins
#  29NOV 2022
#  1. Added functions for VehicleIdle, DrivingMotorsIdle
#  2. Added Events called VehicleIdle. This is cleared here when the Vehicle is identified to be Idle
#  3. Added Variable for VehicleState VEHICLE_IS_IDLING, VEHICLE_IS_INERROR, VEHICLE_IS_BUSY ...
#  4. Added Function SetDriveMotorsToIdle, called when the Vehicle control is identified to be idling
#  5. Added code to identify the idling state of the vehicle (done when there are no joystick input in Manual mode)
# 2APR2023 : Added TORQUE mode control for Drive Wheel
# 4APR2023 : Added 3Wheel Steering Mode
# 6APR2023 : Added BRAKEMODE Function for 3WHEEL Steering Mode
# import gpiozero
# 14APR2023 Morning 11.30 code making idle based on the time value otherwise putting always in closedloop
#  vibration while putting in closed loop., Velocity gain value was changed 
# 17APR2023 code changed for synchronous drivein the steering wheel.and three wheel steering is also implemented for testing 
# 20APR2023 : Added Automatic Mode to the functionality
# 20APR2034 : canbus buffer size added to remove errors 
# TODO : Set odrive Watch Dog time to timeout/goto idle after SoftwareTimeOutErrorTime = 100 milliseconds
# TODO : Add timeout Error When ODRIVE fails to send HeartBeat Message
# TODO : Add ActionOnError function 
# TODO : 22APR2023 : Tested the Automatic and Manual Mode. Error steering (in lab) motor when switching from Automatic to Normal Mode control
# TODO : 25APR2023 : Line no. 567 for the above message to kill.
# 16May2023 : Line no.607, 618 and some more lines were added for the change of control mode.
#             Linesno.385 and 392 were chanded to move the vhicle by speed
#             based on line no.385 and 392 some functions were introduced in Vehiclecanbusinterface,py
#             Introduced Trap TrajValues in the code for variable speed in different mode
#             In ManualMode (ie when Automatic Switch is OFF and LEFT/RIGHT control is not set)
#             the vehicle will be completely set in VelocityControl Mode to speed the movement of Vehicle
#             The JoyStick Mode when operated in position control is extremely slow movement
#             TODO : Velocity Control we are not sure of the Torque when going up or down a slope
# 17MAY2023 : The DriveMotors are explicitly set to PositionControl in all Modes
#           : The steering Motor trap values at set at initially
# 18MAY2023 : i) Steering Control is changed from 2 wheel to 3 wheel steering
#             ii) Pivot mode added not tested yet
# 19MAY2023 :
#            i) Adding Joystick Mode in AutomaticMode to operate in SlowSpeedMode
#           ii) LEFT, RIGHT and Neutral direction will work in AutomaticMode
#           iii) In ManualMode Joystick operation is in highspeed mode
#            iv) LEFT and RIGHT direction switch in Manual Mode will pivot when moved with Joystick
#             v) TODO : Software bug when wheels slip or skid and vehicle forward/backward control faile
#           vi) Reset of odrive periodically in closed loop control to eliminate the drive wheel skid, slip and stuck conditions of the wheel
#           vii)TODO : 3 Wheel steering requires precise value for distance for each wheel
# 23MAY2023 i) Fixed Gear Ratio for accurate distance movement
#           ii) Setting Of Velocity Limit and Trap Trajectory was not done  in MANUAL mode, added code
#           iii) Position estimate of Each Wheel is done separately and used for MoveVehicle. 
#                Previously we were using one position estimate and using it for all the other wheels which will create erroneous
#                conditions when the wheel slips or skids
# 24MAY2023 i) MaxCurrentLimit and MaxVelocityLimit explicity in the Initialise()
# 26MAY2023 i) Added logs to a file at /home/ubuntu/pragati/logs/VehicleControl.logs , all prints goes to this file and STDOUT
#           ii) TODO : Adding Function called StartCondition which reads MARCStartSignal (IPC) or in DebugMode Read StartButton 
#           iii)TODO : Adding Function called ShutDownCondition which reads MARCShutDownSignal (IPC) or in DebugMode Read ShutDownButton  
# 27MAY2023 i) Adding Timing information to the logs. The following Timers are added when Mode is Switched.
#                a. Time Added when Mode Switch happens ( Helps is characterising Moving in a line in automatic mode and navigation in manual mode)
#                b. Timer added for StartButton tasks (useful for checking time for moving one Step)
#                c. Timer Added when for every Direction (LEFT) Change in Automatic Mode.
# 31MAY2023 i)re installed the canbus tools for the new version of ubuntu
#           ii) the float value is taken for the position calculation in ODrive helper function. 
# 6JUN2023 i)2 wheel ackermann steering with the IMU controls on line no. 728
# 9JUN2023 : Added Direct call to SetSteeringMotorCurrentAndVelocityLimits(CurrentLimit,VelocityLimit)
#           Temporary disabled IMU function to run code in the field without the IMU


import os
import can
#import cantools
import numpy
import binascii
import struct
import time
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO librarya

#import PragatiConfig
import JoyStickControlWithAdaFruitDriver
import VehicleCanBusInterface 
import logging
IMUEnable = False
if (IMUEnable) :
  import IMUReadExperimenti #Commenting out for running in fiel

MaximumMovePosition =3.14*24*25.4 # one rotation of wheel in mm
MaximumVehicleTorque = 20 # N/m
MaximumVehicleVelocity = 5000 # 1000m/hour
MinimumVehicleDistanceResolution  = 100 #in mm
MinimumVehicleSteeringAngle = 0.02 # Steering Angle whichis approximately equal 7.5 degree 
MaximumSteeringAngle  = 45 # degrees is rotation MaximumSteeringAngle  = 70/360 # 70 degrees in rotation 
MaximumSteeringTurnOfWheel = MaximumSteeringAngle/360 
# TODO Move these to a Global File called PragatiConfig
# Pin Definition for different Functions of Vehicle
#DirectionSwitch   =  16  # 1 forward, 0 reverse direction move in Manual 
#StopButton        =  6   # Stops the vehicle, should act as an electric brake 
#AutoManualSwitch  =  26  # Enable is Auto, Disabled is Manual
#ArmStart          =  5  # Used in Manual Mode to start Arm operation PressButton

VEHICLE_IN_MANUAL_MODE = 0x1111
VEHICLE_STOP_REQUEST = 0x9999
VEHICLE_IN_AUTOMATIC_MODE = 0x5555
VEHICLE_IS_INERROR = 0x3333
VEHICLE_IS_IDLING = 0x0000
VEHICLE_IS_BUSY = 0xEEEE
SYSTEMRESETMODE = 0xDEAD
MANUALMODE_LEFTDIRECTION = 0X1114
MANUALMODE_RIGHTDIRECTION = 0X1113
NONBRAKE_MANUAL_MODE = 0x1112
SYSTEM_MODE_UNKNOWN = 0x7777
VEHICLE_IN_BRAKESWITCH_MODE = 0XBBBB
SystemModeValue = SYSTEM_MODE_UNKNOWN 
PreviousSystemModeValue = SYSTEM_MODE_UNKNOWN 

PIVOT_LEFT = 0xAAAA
PIVOT_RIGHT = 0x5555
PIVOT_NONE = 0x0000

ButtonPressed = 0XABCD
ButtonNotPressed = 0XDCBA


CurrentLimit=50
VelocityLimit=8

DriveVelocityLimit=0
DecelLimit=0
AccelLimit=0
LowSpeed =2
HighSpeed=3.5
LowAccel=0.5
HighAccel=1
LowDecel=0.5
HighDecel=1
SteeringVelocityLimit=5
SteeringDecelLimit=2
SteeringAccelLimit=2
 
VehicleIsSetToIDLE = True
XValue = 0 
YValue = 0 
LastXValue = XValue 
LastYValue = YValue 
JoyStickMidValue = 512
JoyStickMaxValue = 1023
JoyStickMinValue = 0 
JoyStickResolution = 220
IdlingTime = 0.5  # if there is No activity given to the motors , then it is use to the Idle the motors


#ControlMode= CONTROL_MODE_UNKNOWN

# MARC Control Pins and RaspberryPI GPIO signals
DirectionLeft       = 6 # // Input Pin
DirectionRight      = 16 #//Input pin
AutomaticMode       = 26 # // Input Pin
VehicleStopButton   = 13 # // Input Pin
SystemReset         = 13 # // Input Pin
ArmShutDown         = 4  # // Input Pin
ArmStart            = 5 # // Input Pin
BrakeSwitch         = 12 # //Input pin

# Marc Control Output Pins and RaspberryPI GPIO signals
GreenLed      = 27 #
YellowLed     = 22 #
RedLed        = 17 #
FAN           = 24 #
ErrorLED      = 23 #

global LastOperationTime 

DEBUGLOG = True
DEBUGLOG1 = False

LOGFILEENABLED = True
LOGFILEPATH = "/home/ubuntu/pragati/outputs/VehicleControl.log"
global root_logger

if (LOGFILEENABLED ) :
  #logging.basicConfig(filename=LOGFILEPATH )
  root_logger= logging.getLogger()
  root_logger.setLevel(logging.INFO) # or whatever
  handler = logging.FileHandler(LOGFILEPATH, 'a+', 'UTF-8') # or whatever
  formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)s : %(message)s', datefmt='%d-%b-%Y:%H:%M:%S') # or whatever
  handler.setFormatter(formatter) # Pass handler as a parameter, not assign
  root_logger.addHandler(handler)  
 
 # logging.basicConfig(filename=LOGFILEPATH, filemode='a', force = 'True', format='%(asctime)s %(name)s %(levelname)s : %(message)s', datefmt='%d-%b-%Y:%H:%M:%S', level=logging.INFO)

def printINFO(txtmsg) :
   global root_logger
   if (DEBUGLOG) :
      print(txtmsg)
   if (LOGFILEENABLED):
     root_logger = logging.getLogger(__name__)
     root_logger.info(txtmsg)
# end of printINFOINFO

def printDEBUG(txtmsg) :
   global root_logger
   if (DEBUGLOG) :
      print(txtmsg)
   if (LOGFILEENABLED):
     logger = logging.getLogger(__name__)
     logger.debug(txtmsg)
# end of printDEBUG


def printWARN(txtmsg) :
   global root_logger
   if (DEBUGLOG) :
      print(txtmsg)
   if (LOGFILEENABLED):
     logger = logging.getLogger(__name__)
     logger.warning(txtmsg)
# end of  printWARN

def printERROR(txtmsg) :
   global root_logger
   if (DEBUGLOG) :
      print(txtmsg)
   if (LOGFILEENABLED):
     logger = logging.getLogger(__name__)
     logger.error(txtmsg)
# end of  printERROR

def CloseLogFile() :
   global root_logger
   if (LOGFILEENABLED): 
      try :
         root_logger.shutdown() 
      except :
         print("Error , Not able to run logging.shutdown") 

def LogTime(TxtMsg, StartTime) :
   global root_logger
   msg = TxtMsg + f"{( (time.time() - StartTime)* 1000) }" + " MilliSec"
   if (DEBUGLOG) :
      print(msg)
   if (LOGFILEENABLED):
     root_logger = logging.getLogger(__name__)
     root_logger.info(msg)
#endof def LogTime(TxtMsg, StartTime) :

DEBUGJOYSTICK = True
def printINFOJOYSTICK (txtmsg) :
   if (DEBUGJOYSTICK) :
      printINFO(txtmsg)
# end of printINFOJOYSTICK 

def ActionOnError():
    VehicleCanBusInterface.RebootAllOdrives()

       
def xxSetupIOPins():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GreenLed  ,GPIO.OUT ) 
    GPIO.setup(YellowLed ,GPIO.OUT ) 
    GPIO.setup(RedLed    ,GPIO.OUT ) 
    GPIO.setup(FAN       ,GPIO.OUT )
    GPIO.setup(ErrorLED  ,GPIO.OUT )
    GPIO.setup(DirectionLeft       ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(AutomaticMode,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(SystemReset,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(ArmShutDown     ,GPIO.IN , pull_up_down = GPIO.PUD_UP) 
    GPIO.setup(ArmStart        ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(DirectionRight       ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(BrakeSwitch       ,GPIO.IN , pull_up_down = GPIO.PUD_UP) 

def TestOutputPins():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GreenLed  ,GPIO.OUT ) 
    GPIO.setup(YellowLed ,GPIO.OUT ) 
    GPIO.setup(RedLed    ,GPIO.OUT ) 
    GPIO.setup(FAN       ,GPIO.OUT )
    GPIO.setup(ErrorLED  ,GPIO.OUT )
    for OutputPin in [GreenLed, YellowLed, RedLed, FAN,ErrorLED]:
            GPIO.output(OutputPin, GPIO.HIGH)  
            time.sleep(4)
            GPIO.output(OutputPin, GPIO.LOW)  
            time.sleep(4)
            GPIO.output(OutputPin, GPIO.HIGH)  

def TestInputPins():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DirectionLeft       ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(AutomaticMode   ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(SystemReset,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(ArmShutDown     ,GPIO.IN , pull_up_down = GPIO.PUD_UP) 
    GPIO.setup(ArmStart        ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(DirectionRight       ,GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
    GPIO.setup(BrakeSwitch       ,GPIO.IN , pull_up_down = GPIO.PUD_UP) 

    printINFO(" DirectionLeft  (L2) Value            :  " + str( GPIO.input(DirectionLeft) ))
    printINFO(" AutomaticMode  (L1) Value         : "  + str( GPIO.input(AutomaticMode) ))
    printINFO(" SystemReset(reset) Value  : " + str( GPIO.input(VehicleStopButton) ))
    printINFO(" ArmShutDown  (Shutdown)    Value      :  "  + str( GPIO.input(ArmShutDown     )))
    printINFO(" ArmStart Value  (start)            :  "  + str( GPIO.input(ArmStart)))
    printINFO(" DirectionRight  (L3)            :  "  + str( GPIO.input(DirectionRight)))
    printINFO(" BrakeSwitch  (servoPwm)            :  "  + str( GPIO.input(BrakeSwitch)))
   
import numpy as np

def reject_outliers(data, m=2):
   d = np.abs(data-np.median(data))
   mdev=np.median(d)
   s = d/mdev if mdev else 0
   return data[s < m]

def FilteredJoystick_Value_test():
   ListXvalue=[]
   ListYvalue=[]
   for i in range (0,20):
     XValue,YValue= JoyStickControlWithAdaFruitDriver.ReadJoyStickXY()
     ListXvalue.append(int(XValue/100)) 
     ListYvalue.append(int(YValue/100))
     #ListXvalue.append(int(XValue)) 
     #ListYvalue.append(int(YValue))
#     time.sleep(0.1)
   #print(ListXvalue)
   #print(ListYvalue)
   Xvalue_List = Counter(ListXvalue)
   Filtered_XValue= (Xvalue_List.most_common()[0][0])*100 
   Yvalue_List = Counter(ListYvalue)
   Filtered_YValue= (Yvalue_List.most_common()[0][0])*100
   #print(Filtered_XValue)
   #print(Filtered_YValue)
   return Filtered_XValue,Filtered_YValue

def TestFilteredJoyStickValue():
  while True:
   #Initialise()
   #print("Reading X and Y Values of Joystick")
   XValue , YValue = FilteredJoystick_Value_test()
   print(f"XValue: {XValue}, YValue: {YValue}\n")
#   time.sleep(1)

def FilteredJoystick_Values():
  for i in range (0,10) :
     XValue,YValue= JoyStickControlWithAdaFruitDriver.ReadJoyStickXY()
     ListXvalue.append(XValue) 
     ListYvalue.append(YValue)
  CleanXList = reject_outliers(ListXValue)  
  CleanYList = reject_outliers(ListYValue)  
  return (np.mean(CleanXlist), np.mean(CleanYlist))


def VehicleControlWithJoyStick():
  global LastXValue 
  global LastYValue 
  global LastOperationTime 
  global SystemModeValue 
  global PreviousSystemModeValue 
  global VehicleIsSetToIDLE 
  
  #TODO mode to be handled for 
  #VEHICLE_IN_AUTOMATIC_MODE 
  # NONBRAKE_MANUAL_MODE MANUALMODE_LEFTDIRECTION MANUALMODE_RIGHTDIRECTION
  # BRAKE_MANUAL_MODE 
#  if ( SystemModeValue == VEHICLE_IN_BRAKESWITCH_MODE):
 #   return True
  NoAction = -1
  VehicleIdling = 333
  JoyStickTimerStart = time.time()

  #XValue, YValue = JoyStickControlWithAdaFruitDriver.ReadJoyStickXY()
  XValue, YValue = FilteredJoystick_Value_test()
  #print(f"XValue: {XValue}, YValue: {YValue}\n")
  #YValue =  (YValue -JoyStickMaxValue ) # DriveMotorDirection is now configured incorrectly TODO to fix the problem in HW and change in SW
 
  #printINFOJOYSTICK("**** XValue,LastXValue : " + str(XValue) + " " + str( LastXValue) + " YValue, LastYValue : "+ str(YValue) +  " " +str(LastYValue) ) #ConditionToMove =  ( (XValue - LastXValue) > 50) or (  (YValue - LastXValue) > 50  ) 
  # Below condition identifies when the user input on joystick is considered to be idling
  #printINFOJOYSTICK( "Time to Read JoyStick : {}".format(1000*(time.time() - JoyStickTimerStart)) )
  LogTime( "Time to Read JoyStick : {}",JoyStickTimerStart)
  # Reset YValue if its value is below the JoyStickResolution from JoyStickMidValue 
  # This is to avoid small YValues create a noisy input and 
  # render the controlling of the vehicle position difficult in ManualMode
  if (abs(YValue - JoyStickMidValue) < JoyStickResolution) : 
    YValue = JoyStickMidValue 

 # that is the user is not actively controllling the vehicle  
  if  (   (abs(XValue - JoyStickMidValue     ) < JoyStickResolution ) and
          (abs(LastXValue - JoyStickMidValue ) < JoyStickResolution )  and
          # Added the following Line to bring the Vehicle to Zero position on PositionControlMode 19MAY2023 
          # This logic may work only for Position ControlMode and not for Torque or Velocity Control Mode
          (abs(LastYValue - JoyStickMidValue ) < JoyStickResolution )  and
          (abs(YValue - JoyStickMidValue     ) < JoyStickResolution ) ) :
      #printINFOJOYSTICK( "JoyStick Value within Idle Conditions")
      printINFO( "JoyStick Value within Idle Conditions")
      CurrentTime = time.time()
      #printINFOJOYSTICK( " CurrentTime :{} LastOperationTime : {} IdlingTime :{}".format(CurrentTime, LastOperationTime, IdlingTime))
      LastXValue = XValue 
      LastYValue = YValue 
      if (CurrentTime - LastOperationTime) > IdlingTime :
  
         VehicleIsSetToIDLE = True
         #printINFO( "Idling time Hit") 
         if ( ( SystemModeValue == MANUALMODE_LEFTDIRECTION) or ( SystemModeValue == MANUALMODE_RIGHTDIRECTION) ):
            VehicleCanBusInterface.SetDriveMotorsToIdle()
            #printINFOJOYSTICK( "Time to SetDriveMotorsTo IDLE : {}".format(1000*(time.time() - JoyStickTimerStart))) 
            LogTime( "Time to SetDriveMotorsTo IDLE : ",JoyStickTimerStart) 
            printINFO("Drivemotor is Idle")
            return (VEHICLE_IS_BUSY) # TODO Change to DRIVE_MOTOR_IS_IDLING
         else:
            # ALl other Conditions putting the vehicle in Idling Mode
            printINFO(" Alll Motors are driven to Idle")
            VehicleCanBusInterface.SetVehicleToIdle()
            #printINFOJOYSTICK( "Time to VehicleTo IDLE : {}".format(1000*(time.time() - JoyStickTimerStart))) 
            LogTime( "Time to VehicleTo IDLE : ",JoyStickTimerStart) 
            return (VEHICLE_IS_IDLING) # Let Vehicle not change state
      else : return ( VEHICLE_IS_BUSY) 

  #printINFOJOYSTICK( "Time to Decide on NON IDLING CONDITION: {}".format(1000*(time.time() - JoyStickTimerStart))) 
  LogTime( "Time to Decide on NON IDLING CONDITION: {}",JoyStickTimerStart) 
  #printINFOJOYSTICK( "*** Motors should be OUT OF IDLE")
  if (VehicleIsSetToIDLE == True ) :
      VehicleCanBusInterface.SetDriveMotorsToClosedLoopControl()
      VehicleCanBusInterface.SetSteeringMotorsToClosedLoopControl()
      VehicleIsSetToIDLE = False 
      #printINFOJOYSTICK( "Time to GetOutOf IDLE Condition: {}".format(1000*(time.time() - JoyStickTimerStart)))
      LogTime( "Time to GetOutOf IDLE Condition: {}",JoyStickTimerStart)
      
#********THIS CONDITION IS TO BE CHECHKED FOR JOYSTICK FAILURE************
 # if not ( (abs(XValue - LastXValue) > 50) or (  abs(YValue - LastYValue) > 50  )) :
   #   print("Vehicle Is Busy")
  #    return ( VEHICLE_IS_BUSY ) # Let Vehicle not change state

  if (XValue > 1023) or (XValue < 0) :
     printINFO("Erroneous Value of X \n") 
     printINFO("Not using this value\n") 
     return(VEHICLE_IS_INERROR)
  if (YValue > 1023) or (YValue < 0) :
     printINFO("Erroneous Value of YValue \n") 
     printINFO("Not using this value\n") 
     return(VEHICLE_IS_INERROR)

###   ADC reads a Minimum of 0 and a Maximum value of 1023 (looks like 10 bit ADC)   
#### Value         Angle      NoOfRotations
#     0             -90        -0.25
#    500             0          0.0 
#    1000           +90        +0.25  

#### Value               Distance(mm)
#     0                    -500
#    500                    0.0 
#    1000                  +500  

  #SteeringWheelRotation =  MaximumSteeringTurnOfWheel * (XValue - 500 )/500
  SteeringWheelRotation =  MaximumSteeringTurnOfWheel * (XValue - JoyStickMidValue )/JoyStickMidValue 
  if (SteeringWheelRotation > MaximumSteeringTurnOfWheel  )   : SteeringWheelRotation =  MaximumSteeringTurnOfWheel 
  elif (SteeringWheelRotation < -( MaximumSteeringTurnOfWheel )) : SteeringWheelRotation = -(MaximumSteeringTurnOfWheel)
  # Maximum and Minimum Steering Angle is 
  SteeringAngleInDegree = (SteeringWheelRotation/MaximumSteeringTurnOfWheel) * MaximumSteeringAngle    
  # Move only if there is more than 100mm to move 
  #DistanceToMoveInMM = -1* 3 * (YValue - 500 ) 
  #RequiredVehicleVelocity =  ((YValue - 500 ) / 500) * MaximumVehicleVelocity
  RequiredVehicleVelocity =  ((int) ((YValue - JoyStickMidValue )) /JoyStickMidValue  ) * MaximumVehicleVelocity
  if (RequiredVehicleVelocity > MaximumVehicleVelocity)   : RequiredVehicleVelocity = MaximumVehicleVelocity
  elif  (RequiredVehicleVelocity < -( MaximumVehicleVelocity)) : RequiredVehicleVelocity = -(MaximumVehicleVelocity)
#  printINFO("Moving Vehicle by " + str(RequiredVehicleVelocity) + " in m/s , Steering by " +str(SteeringAngleInDegree) + "  in degree Angle" ) 
 
  # Move only if there is more than 100mm to move 
  # TODO remove floating Point Math here. Make all value to Integer s in RequiredVehicleVelocity
  RequiredDistanceToMoveInMM =  MaximumMovePosition * ( (YValue - JoyStickMidValue )/  JoyStickMidValue) 
  if (RequiredDistanceToMoveInMM > MaximumMovePosition )   : RequiredDistanceToMoveInMM  = MaximumMovePosition 
  elif  (RequiredDistanceToMoveInMM < -( MaximumMovePosition )) : RequiredDistanceToMoveInMM  = -(MaximumMovePosition)

  RequiredVehicleTorque =  (((int) (YValue - JoyStickMidValue )) / JoyStickMidValue ) * MaximumVehicleTorque
  if (RequiredVehicleTorque > MaximumVehicleTorque)   : RequiredVehicleTorque = MaximumVehicleTorque
  elif  (RequiredVehicleTorque < -( MaximumVehicleTorque)) : RequiredVehicleTorque = -(MaximumVehicleTorque)
#  printINFO("Moving Vehicle by " + str(RequiredVehicleTorque) + " in torque nm , Steering by " +str(SteeringAngleInDegree) + "  in degree Angle" ) 
  
  #print(f"Moving Vehicle by Distance {DistanceToMoveInMM} in mm , Steering by {SteeringAngleInDegree} in degree Angle\n" ) 
  #if( abs(DistanceToMoveInMM)   > MinimumVehicleDistanceResolution ) : 
  #     VehicleCanBusInterface.MoveVehicle(DistanceToMoveInMM)
  #     VehicleCanBusInterface.MoveVehicleSpeedAngle(DistanceToMoveInMM, SteeringWheelRotation)
  #if(abs(SteeringWheelRotation) > MinimumVehicleSteeringAngle ) :   VehicleCanBusInterface.SteerVehicle(SteeringWheelRotation)
  #VehicleCanBusInterface.SteerVehicle(SteeringWheelRotation)
  #VehicleCanBusInterface.MoveVehicleBySpeedAndAngle(RequiredVehicleVelocity, SteeringWheelRotation)   

  #TODO BRAKE_MANUAL_MODE 
  #if (SystemModeValue == BRAKE_MANUAL_MODE):
  # TODO ..
  #if ( SystemModeValue == VEHICLE_IN_BRAKESWITCH_MODE):
  #   VehicleCanBusInterface.MoveVehicle(RequiredVehicleTorque)   
  
  #printINFOJOYSTICK( "Time for Computing Steer and Distance Value:  {}".format(1000*(time.time() - JoyStickTimerStart))) 
  LogTime( "Time for Computing Steer and Distance Value: ",JoyStickTimerStart) 
    
  VehicleDirection = FilteredGetDirection()
  if (VehicleDirection == NONBRAKE_MANUAL_MODE) :
     VehicleCanBusInterface.MoveVehicle(RequiredDistanceToMoveInMM,SteeringWheelRotation)   
  else :
     VehicleCanBusInterface.MoveDriveMotorsIncremental( RequiredDistanceToMoveInMM ) 
  ''''     
  if ( ( SystemModeValue == MANUALMODE_LEFTDIRECTION) or ( SystemModeValue == MANUALMODE_RIGHTDIRECTION) ):
     #TODO Check whether the Steering Wheels have reached the desired location, then move the drivewheels
     #SetVehicleInBrakeMode()
     VehicleCanBusInterface.MoveDriveMotorsIncremental( RequiredDistanceToMoveInMM ) 
     printINFOJOYSTICK( "Time For MoveDriveMotorsIncremental Instructions :  {}".format(1000*(time.time() - JoyStickTimerStart))) 
     #VehicleCanBusInterface.MoveVehicleBySpeed(RequiredVehicleVelocity)   
     #VehicleCanBusInterface.MoveVehicleByTorque(RequiredVehicleTorque)
  else:
     #Following is for NONBRAKE_MANUAL_MODE
     #VehicleCanBusInterface.MoveVehicleByTorqueAndAngle(RequiredVehicleTorque,SteeringWheelRotation)   
     #VehicleCanBusInterface.MoveVehicleByPositionAndAngle(RequiredDistanceToMoveInMM,SteeringWheelRotation)   
     #SetVehicleInBrakeMode()
     #VehicleCanBusInterface.MoveVehicleBySpeedAndAngle(RequiredVehicleVelocity, SteeringWheelRotation)   
     VehicleCanBusInterface.MoveVehicle(RequiredDistanceToMoveInMM,SteeringWheelRotation)   
     printINFOJOYSTICK( "Time For MoveVehicle Instructions :  {}".format(1000*(time.time() - JoyStickTimerStart))) 
     #VehicleCanBusInterface.MoveDriveMotorsIncremental( RequiredDistanceToMoveInMM ) 
  ''' 

  ''' 
  VehicleCanBusInterface.ReadAxisError(0)
  VehicleCanBusInterface.ReadAxisError(1)
  VehicleCanBusInterface.ReadAxisError(2)
  VehicleCanBusInterface.ReadAxisError(3)
  VehicleCanBusInterface.ReadAxisError(4)
  VehicleCanBusInterface.ReadAxisError(5)
  '''
  LastXValue = XValue 
  LastYValue = YValue
  LastOperationTime = time.time()
  return( VEHICLE_IS_BUSY ) 


def VehicleMovingLeft():
  VehicleCanBusInterface.SetSteeringMotorsToClosedLoopControl()
  VehicleCanBusInterface.MoveSteeringMotorsTo90degreerotationLeft()
  time.sleep(10)
def VehicleMovingRight():
  VehicleCanBusInterface.SetSteeringMotorsToClosedLoopControl()
  VehicleCanBusInterface.MoveSteeringMotorsTo90degreerotationRight()
  time.sleep(10)

def BrakeButton_Pressed():
  VehicleCanBusInterface.SetDriveMotorsToIdle()
  VehicleCanBusInterface.SetDriveMotorsToPositionControlMode()
  VehicleCanBusInterface.SetDriveMotorsToClosedLoopControl()
  return VEHICLE_IN_BRAKESWITCH_MODE

def SetVehicleInBrakeMode():

  VehicleCanBusInterface.SetDriveMotorsToPositionControlMode()
  VehicleCanBusInterface.SetDriveMotorsToClosedLoopControl()
  return VEHICLE_IN_BRAKESWITCH_MODE


def StopVehicle():
  printINFO("IN DEBUG MODE, Not Implemented VehicleStop\n")
  VehicleCanBusInterface.SetVehicleToIdle()
  return (VEHICLE_STOP_REQUEST) 

'''ASSERTVALUE = LOW'''

def CanbusDisconnectionError():
  CanBusID=0
  MaxCount=1
  vehicleCanBusInterface.ReadCanIDfromHeartbeatMsg(CanBusID,MaxCount)
  exit()

def GetDirection():
   if ((GPIO.input(DirectionRight) == GPIO.HIGH)): # Direction and Direction_Two cannot be high simultaneously
      #printINFO("vehicle moving right")
      #printINFO("3 wheel mode")
      return MANUALMODE_RIGHTDIRECTION
   if ((GPIO.input(DirectionLeft) == GPIO.HIGH)):
      #printINFO("vehicle moving left")
      #printINFO("3 wheel mode")
      return MANUALMODE_LEFTDIRECTION
   return NONBRAKE_MANUAL_MODE  # ADDED TO COMPLETE functionality
#End of def GetDirection():

from collections import Counter
def FilteredGetDirection() :  
   Value1 =  GetDirection()
   Value2 =  GetDirection()
   Value3 =  GetDirection()
   Value4 =  GetDirection()
   Value5 =  GetDirection()
   valueList = [Value1,Value2,Value3,Value4,Value5]
   values = Counter(valueList)
   return( values.most_common()[0][0]) 

def FilteredVehicleControlMode() :  
   global PreviousSystemModeValue
   global SystemModeValue
   PreviousSystemModeValue = SystemModeValue
   Value1 =  VehicleControlMode()
   Value2 =  VehicleControlMode()
   Value3 =  VehicleControlMode()
   Value4 =  VehicleControlMode()
   Value5 =  VehicleControlMode()
   valueList = [Value1,Value2,Value3,Value4,Value5]
   values = Counter(valueList)
   return( values.most_common()[0][0]) 
    
def VehicleControlMode():
   #print("in DebugMode not checked Manual, StopRequest  \n")
   #print(" Now hacked to VEHICLE_IN_MANUAL_MODE\n")
   #TODO CHECK THE SYSTEMRESET   BUTTON ACTIVE VALUE 

   if (GPIO.input(SystemReset) == GPIO.HIGH):
      #print("reset mode")
      return SYSTEMRESETMODE 
   #TODO Define VEHICLE_STOP_REQUEST
   # return VEHICLE_STOP_REQUEST

#   if (GPIO.input(BrakeSwitch) == GPIO.LOW):
 #     printINFO("VEHICLE_IS_IN_BRAKE_CONTROL \n")
      #print("brake button")
  #    return VEHICLE_IN_BRAKESWITCH_MODE 

   if (GPIO.input(AutomaticMode) == GPIO.HIGH):
      printINFO("VehicleControl.VehicleControlMode :: VEHICLE_IN_AUTOMATIC_MODE \n")
      #print("automatic mode")
      return VEHICLE_IN_AUTOMATIC_MODE 
      
   if ((GPIO.input(DirectionRight) == GPIO.HIGH)): # Direction and Direction_Two cannot be high simultaneously
      #print("vehicle moving right")
      #print("3 wheel mode")
      return MANUALMODE_RIGHTDIRECTION

   if ((GPIO.input(DirectionLeft) == GPIO.HIGH)):
      #print("vehicle moving left")
      #print("3 wheel mode")
      return MANUALMODE_LEFTDIRECTION

   #TODO : define BRAKE_MODE 
   # return BRAKE_MANUAL_MODE
   #print("2 wheel mode")
   return NONBRAKE_MANUAL_MODE  

#def Controlmode 
#  global

def Initialise():
   global LastXValue 
   global LastYValue 
   global LastOperationTime 
   global CurrentLimit
   global VelocityLimit
 
   InitialisationStartTime = time.time()
   xxSetupIOPins()
   #LastXValue ,LastYValue = JoyStickControlWithAdaFruitDriver.ReadJoyStickXY()
   VehicleCanBusInterface.Initialise()

   VehicleCanBusInterface.SetDriveMotorCurrentAndVelocityLimits(CurrentLimit,VelocityLimit)
   VehicleCanBusInterface.SetSteeringMotorCurrentAndVelocityLimits(CurrentLimit,VelocityLimit)
   SettingTrapTrajValues()
   time.sleep(1)
   # TODO Velocity Values of the steering Motors also to be Set 
   VehicleCanBusInterface.SetDriveMotorsToPositionControlMode()
   VehicleCanBusInterface.SetDriveMotorsToClosedLoopControl()
   # TODO write the software code for SetSteeringMotorsToPositionControlMode()
   #VehicleCanBusInterface.SetSteeringMotorsToPositionControlMode()
   VehicleCanBusInterface.SetSteeringMotorsToClosedLoopControl()
   VehicleIsSetToIDLE = False 
   JoyStickControlWithAdaFruitDriver.Initialise()
   LastOperationTime = time.time()
   LogTime("Time to VehicleControl Initialise() : ", InitialisationStartTime )
#End of def Initialise():

def VehicleShutDown():
   VehicleShutDownStartTime = time.time()
   printINFO("Vehicle is ShuttingDown")
   # TODO: do all the shutdown tasks
   printINFO("Completed Shutting Down of Vehicle")
   LogTime("Time to ShutDown : ", VehicleShutDownStartTime)
#End of def VehicleShutDown()

def ButtonStatus():
   if ((GPIO.input(ArmStart) == GPIO.HIGH)):
      return ButtonPressed
   else:
      return ButtonNotPressed
   
def FilteredStartButtonValue() :  
   Value1 = ButtonStatus()
   Value2 = ButtonStatus()
   Value3 = ButtonStatus()
   Value4 = ButtonStatus()
   Value5 = ButtonStatus()
   valueList = [Value1,Value2,Value3,Value4,Value5]
   values = Counter(valueList)
   return( values.most_common()[0][0]) 
 


#TODO Check for the Vehicle Motors to complete Movement operations 
#      If it is completed within a stipulated Time return True else return False 
def  CheckForOperationToComplete() :
    WaitTime = 10 # Compute the Wait Time based on Actual task
    time.sleep(WaitTime)
    # ifOperationCompleted():
    return ( True)
    # else : return (False)
#End of def WaitForOperationToComplete() :
'''
def Filtered_IMUValue():
   ListIMUvalue=[]
   for i in range (0,20):
     IMUValue = IMUReadExperiment.ReadIMU()
     ListIMUvalue.append(IMUValue) 
   print(ListIMUvalue)
   IMUvalue_List = Counter(ListIMUvalue)
   Filtered_IMUValue= (IMUvalue_List.most_common()[0][0]) 
   print("Filtered_IMUValue :",Filtered_IMUValue)
   return Filtered_IMUValue
'''
def AutomaticControlMode():
   global SystemModeValue
   global   PreviousSystemModeValue
   SystemModeValue = FilteredVehicleControlMode()
   ButtonStatus=FilteredStartButtonValue()
   ForwardMovement=500
   BackwardMovement=-500
   printINFO("VehicleCantrol.AutomaticControlMode :: Automtic Movement")
   VehicleDirection = FilteredGetDirection()
   #VehicleCanBusInterface.ReadAxisErrors_AllMotors()
   #VehicleCanBusInterface.ReadMotorErrors_AllMotors() # to check the motorstatus and reset the watchdog timeout limit by sending a new message.

   # If ButtonPressed is true, move FixedDistance or IncrementalStep Mode else use Steering Control
   if (ButtonStatus == ButtonPressed):
      ButtonPressStartTime = time.time()
      printINFO("ButtonPress Automatic Task Started ")
      VehicleCanBusInterface.SetSteeringMotorTrapTrajVelLimit(SteeringVelocityLimit)
      VehicleCanBusInterface.SetSteeringMotorTrapTrajAccelDecelLimits(SteeringAccelLimit,SteeringDecelLimit)
      VehicleCanBusInterface.SetSteeringMotorsToClosedLoopControl()
      if (VehicleDirection ==  NONBRAKE_MANUAL_MODE) :
         VehicleCanBusInterface.MoveAutomaticMovement(ForwardMovement)
         printINFO( "VehicleCantrol.AutomaticControlMode :: moving Forward")
      elif (VehicleDirection == MANUALMODE_LEFTDIRECTION) :
         VehicleCanBusInterface.MoveAutomaticMovement(BackwardMovement)
         printINFO( "VehicleCantrol.AutomaticControlMode :: moving Backward")
      # THIS PART OF THE CODE IS ONLY IF IMU is Activated 
      '''      
      # Reset the IMU before start reading the Value so that it starts at Zero      
      DesiredYawLeftAngle = 0.0  # Assuming at Start or Reset the YawAngle is Zero
      DesiredYawRightAngle = 360.0  # Assuming at Start or Reset the YawAngle is Zero
      TimeForVehicleToCompleteAStep = 10000 # milliseconds , This has to be computed based on the distance
      AutomaticTimeStart = time.time()
      IterationTime = 0.5 # seconds
      MaximumSteeringAngle  = 30 # degrees is rotation MaximumSteeringAngle  = 70/360 # 70 degrees in rotation 
      MaximumSteeringAngleNegative  = -30 # degrees is rotation MaximumSteeringAngle  = 70/360 # 70 degrees in rotation 
      # or Check if the vehicleIsCompleted the Drive Motion such as CheckForOperationToComplete
      while ( not ( ((time.time() - AutomaticTimeStart) * 1000) > TimeForVehicleToCompleteAStep) ) :
         CurrentIMUValue = Filtered_IMUValue()
         if(CurrentIMUValue>180):
            IMUValueDifference =  CurrentIMUValue - DesiredYawRightAngle
         elif(CurrentIMUValue<180):
            IMUValueDifference =  CurrentIMUValue - DesiredYawLeftAngle
         #TODO Check whether the value is +ve when rotated right and -ve when rotated left
         #TODO DANGER Ensure for the Vehicle +ve steering Angle is towards right and viceversa before Proceeding further
         GearRatioOfSteeringWheel = 50 # TODO definethis as a global value for the vehicle
         AbsoluteRotationValue=-(IMUValueDifference*GearRatioOfSteeringWheel)/360
         print(AbsoluteRotationValue)
         VehicleCanBusInterface.AkermannsSteeringMotorsTo( AbsoluteRotationValue)
         time.sleep(IterationTime)
      '''
      #end of While Loop

      #TODO  # Removing this check for operation
      if( not CheckForOperationToComplete() ) :
          printINFO(" AutomaticControlMode Failed")
          exit()
      LogTime("Time in ButtonPress Automatic Task ", ButtonPressStartTime)
   else :
      # Use JoyStick to control in SLOW speed move
      VehicleState = VehicleControlWithJoyStick()
# END OF AutomaticControlMode
def SettingSteeringTrapTrajValues():
    global SteeringVelocityLimit
    global SteeringDecelLimit
    global SteeringAccelLimit
    VehicleCanBusInterface.SetSteeringMotorTrapTrajVelLimit(SteeringVelocityLimit)
    VehicleCanBusInterface.SetSteeringMotorTrapTrajAccelDecelLimits(SteeringAccelLimit,SteeringDecelLimit)
    
def SettingTrapTrajValues():
    global DriveVelocityLimit
    global DecelLimit
    global AccelLimit
    global LowSpeed 
    global HighSpeed
    global LowAccel
    global HighAccel
    global LowDecel
    global HighDecel

    global SystemModeValue
    global PreviousSystemModeValue 
    SystemModeValue = FilteredVehicleControlMode()
    if  (SystemModeValue == VEHICLE_IN_AUTOMATIC_MODE)  :
       DriveVelocityLimit=LowSpeed
       AccelLimit=LowAccel
       DecelLimit=LowDecel
    #elif (SystemModeValue == NONBRAKE_MANUAL_MODE) :
    else :
       DriveVelocityLimit=HighSpeed
       AccelLimit=HighAccel
       DecelLimit=HighDecel
      
    VehicleCanBusInterface.SetDriveMotorTrapTrajVelLimit(DriveVelocityLimit)
    VehicleCanBusInterface.SetDriveMotorTrapTrajAccelDecelLimits(AccelLimit,DecelLimit)
    VehicleCanBusInterface.SetSteeringMotorTrapTrajVelLimit(SteeringVelocityLimit)
    VehicleCanBusInterface.SetSteeringMotorTrapTrajAccelDecelLimits(SteeringAccelLimit,SteeringDecelLimit)

def Execute():
   global SystemModeValue
   global   PreviousSystemModeValue 
   PreviousDirection = 0
   while True:
      #print( "Waiting for user inputs ")
      # TODO Code for SYSTEMRESETMODE 
      #print( "PreviousSystemModeValue  :", PreviousSystemModeValue)
      CurrentModeOperationStartTime =  VehicleControlExecuteStartTime = time.time()
      #PreviousSystemModeValue = SystemModeValue
      SystemModeValue = FilteredVehicleControlMode()
      #print( "PreviousSystemModeValue  :", PreviousSystemModeValue)
      #print( "SystemModeValue  :", SystemModeValue)

      VehicleDirection = FilteredGetDirection()
      DirectionChange = False 
      if (VehicleDirection != PreviousDirection) : 
        DirectionChange = True 
      PreviousDirection = VehicleDirection 

      ModeChange = False 
      if (SystemModeValue != PreviousSystemModeValue) : ModeChange = True 
      #printINFOJOYSTICK( "Time For SystemModeValue Computation:  {}".format(1000*(time.time() - VehicleControlExecuteStartTime ))) 
      if (SystemModeValue == VEHICLE_IN_AUTOMATIC_MODE) :
         #if ( PreviousSystemModeValue != SystemModeValue ):
         if ( ModeChange or DirectionChange ):
            LogTime("Time in LastMode of Operation Before Switching to AutomaticMode/Direction : ", CurrentModeOperationStartTime)
            AutomaticModeChangeStartTime = time.time()
            CurrentModeOperationStartTime = AutomaticModeChangeStartTime 
            printINFO( " MODE CHANGE IN AUTOMATIC MODE") 
            SettingTrapTrajValues()
            VehicleCanBusInterface.SetDriveMotorsToPositionControlMode()
            #VehicleDirection = FilteredGetDirection()
            ## +90rotation left removed for AutomaticMode, the direction switch is used for Direction when ButtonPressed in Automatic Mode
            #if (VehicleDirection == MANUALMODE_LEFTDIRECTION) :
            #   VehicleCanBusInterface.MoveSteeringMotorsTo90degreerotationLeft()
            #   printINFOJOYSTICK( " Automatic Mode MOVING  Vehicle in LEFT  Direction") 
            if (VehicleDirection == MANUALMODE_RIGHTDIRECTION) :
               VehicleCanBusInterface.MoveSteeringMotorsTo90degreerotationRight()
               printINFO( " Automatic Mode MOVING  Vehicle in RIGHT Direction") 
            else :
               VehicleCanBusInterface.MoveSteeringMotorsTo0degreerotation()
               printINFO( " Automatic Mode MOVING  Vehicle in NEUTRAL Direction") 
            
            printINFO( " Automatic Mode MOVING  Vehicle Direction") 
            #time.sleep(10)
            if( not CheckForOperationToComplete() ) :
               printINFO(" AutomaticControlMode Operation Failed")
               exit()
            LogTime("Time in Automatic Mode (or Direction) Change Task ", AutomaticModeChangeStartTime )
         printINFO( " Executing AutomaticCOntrol Mode") 
         AutomaticControlMode()        
         #printINFO("VehicelControl.Execute:: Vehicle is automatic Mode ")
         #LogTime("Time in Automatic Mode (or Direction) Change Task ", AutomaticModeChangeStartTime )
         #printINFO( " Executing AutomaticCOntrol Mode") 
         #LogTime("Time in AutomaticModeControlMode VehicleControl : ", VehicleControlExecuteStartTime)
      else:
         #if ( PreviousSystemModeValue != SystemModeValue ):
         if ( ModeChange or DirectionChange ):
            LogTime("Time in LastMode of Operation Before Switching to ManualMode or Direction : ", CurrentModeOperationStartTime)
            ManualModeChangeStartTime = time.time()
            CurrentModeOperationStartTime = ManualModeChangeStartTime 
            SettingTrapTrajValues()
            printINFO( " MODE CHANGE IN MANUAL MODE") 
            if (VehicleDirection == MANUALMODE_LEFTDIRECTION):
                VehicleCanBusInterface.SetVehicleToPivot(PIVOT_LEFT)
            elif (VehicleDirection == MANUALMODE_RIGHTDIRECTION) :
                VehicleCanBusInterface.SetVehicleToPivot(PIVOT_RIGHT)
            elif (VehicleDirection == NONBRAKE_MANUAL_MODE) :
                VehicleCanBusInterface.SetVehicleToPivot(PIVOT_NONE)
            printINFO( "Manual Mode PIVOTING  Vehicle Direction") 
            #time.sleep(10)
            if( not CheckForOperationToComplete() ) :
               printINFO(" Manual Mode operation Failed")
               exit()
            LogTime("Time in Manual Mode (or Direction) Change Task ",ManualModeChangeStartTime)
        # USE THE JOYSTICK FOR DRIVE AND STEERING Motors
         #printINFO( " Executing VehicleControlWithJoyStick") 
         VehicleState = VehicleControlWithJoyStick()
         #printINFOJOYSTICK( "Time in : VehicleControlWithJoyStick {}".format(1000*(time.time() - VehicleControlExecuteStartTime ))) 
   # EndOf While loop

       
if __name__== "__main__" :
    Initialise() 
    Execute()
      
    

