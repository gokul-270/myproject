#History
#Last Date When Code Updated : 6JUN2023
# 16MAY2023 : Added the traptrajVel and acceleration values in code for variable speed  purpose.
# 16MAY2023 : Vehicle Steering is using only 2 front wheel equal direction steering instead of ACKERMAN steering
# 17May2023: Added the steering wheel trap traj values
# 24May2023: Added the current limits and velocity limits value along with the check funtion of trap traj values
# 6JUN2023: corrected the calculation of the distance moved
#import gpiozero
import os
import can
import cantools
import numpy
import binascii
import struct
import time
import pigpio
import canlib
import RPi.GPIO as GPIO
import  OdriveCanBusHelperFunctions
 #from gpiozero import Button
# Global Value Definitions
#  5. Added code to identify the idling state of the vehicle (done when there are no joystick input in Manual mode)import RPi.GPIO as GPIO # Import Raspberry Pi GPIO librarya
#DirectionPin = 24
#MovePin = 22
global TimeForUnitDistance
TimeForUnitDistance =  3600 /(1000 * 1000)  # mm  persec ie 1KM/Hr
TimeForFullWheelSteeringRotation = 30   # in Seconds
###### Software Variables
global bus
global db
global msg
global CurrentPos
global IncrementalPos
global TargetPos
SteeringMotorLeft = 1
SteeringMotorRight = 3
SteeringMotorFront = 5 
FrontDriveMotor = 4
LeftBackDriveMotor = 0 
RightBackDriveMotor = 2
# List the active Set of Motors connected
# Software will hang if the motors are listed here but not connected
ActiveMotorList = [SteeringMotorLeft,SteeringMotorRight,SteeringMotorFront,FrontDriveMotor,LeftBackDriveMotor,RightBackDriveMotor ]
#ActiveMotorList = [FrontDriveMotor,SteeringMotorFront ]

WheelDiameter = 24 * 25.4  # in millimeters for a Wheel Diameter of 24 inch
WheelDistancePerRotation = 3.141519 * WheelDiameter

GearRatioLeftBackDriveMotor   = 3*5.8462
#GearRatioLeftFrontDriveMotor  = 15
GearRatioFrontDriveMotor      = 3*5.8462
GearRatioRightBackDriveMotor  = 3*5.8462
GearRatioSteeringMotor = 50 # One rotation of a Motor will rotate by 1/GearRatioSteeringMotor
WheelDistancePerRotationOfMotor = (3.141519 * WheelDiameter)/GearRatioLeftBackDriveMotor

CurrentPos = 0
CurrentSteerPosition = 0.0
StartingPosition = 0

# End Of Global Value Definitions
#import VehicleDefinitions

PIVOT_LEFT = 0xAAAA
PIVOT_RIGHT  = 0x5555
PIVOT_NONE = 0x00000


DEBUGINFO = True
def printDEBUGINFO (txtmsg) :
   if (DEBUGINFO ) :
      print(txtmsg)
# end of printDEBUGINFO 


def CanBusInitialize():
  global bus 
  global db
  os.system('sudo ifconfig can0 down')
  os.system('sudo ip link set can0 type can bitrate 250000')
  os.system('sudo ifconfig can0 up')
  os.system('sudo ifconfig can0 txqueuelen 1000') # this line is added for the BufferSpace storage error
  CanBusEnablePin = 8 # GPIO8 used for CANBUS enable 
  ADCEnablePin = 7 
  GPIO.setmode(GPIO.BCM)  
  GPIO.setup(ADCEnablePin, GPIO.OUT, initial = GPIO.HIGH) # Disable ADC when running Canbus  
  GPIO.output(ADCEnablePin, GPIO.HIGH) # Disable ADC when running Canbus  
  #TODO find the location of this file and then add it to the path of odrive-cansimple.dbc
  # currentDir = os.system.pwd()
  # CanDBPath = currentDir+"/odrive-cansimple.dbc"
  #db = cantools.database.load_file(CanDBPath)
  db = cantools.database.load_file("odrive-cansimple.dbc")
#  db = cantools.database.fopen("home/ubuntu/pragati/src/VehicleControl/odrive-cansimple.dbc","r",encoding)
  #bus = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')
  bus = can.interface.Bus(channel = 'can0', bustype = 'socketcan')

### Command Keywords ###
#BO_ 6 Set_Axis_Node_ID: 8 Vector__XXX
#BO_ 7 Set_Axis_State: 8 Vector__XXX
#BO_ 11 Set_Controller_Mode: 8 Vector__XXX
#BO_ 12 Set_Input_Pos: 8 Vector__XXX
#BO_ 13 Set_Input_Vel: 8 Vector__XXX
#BO_ 14 Set_Input_Torque: 8 Vector__XXX
#BO_ 15 Set_Limits: 8 Vector__XXX
#BO_ 17 Set_Traj_Vel_Limit: 8 Vector__XXX
#BO_ 18 Set_Traj_Accel_Limits: 8 Vector__XXX
#BO_ 19 Set_Traj_Inertia: 8 Vector__XXX
#BO_ 25 Set_Linear_Count: 8 Vector__XXX
#BO_ 26 Set_Pos_Gain: 8 Vector__XXX
#BO_ 27 Set_Vel_gains: 8 Vector__XXX

VehicleControlDebug = False
def   printDebug(msg):
  if (VehicleControlDebug ) :
     print(msg)

# CommandID is the ID of the Command
def GPIOInitialize():
 GPIO.setwarnings(False)
 GPIO.setmode(GPIO.BCM) 
# GPIO.setup(DirectionPin , GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(MovePin , GPIO.IN, pull_up_down=GPIO.PUD_UP)


def MoveAutomaticMovement(AutomaticMovingDistance):
   global StartingPosition
   MotorPositionEstimate, CurrentVelocity = OdriveCanBusHelperFunctions.GetPositionAndVelocity(bus, LeftBackDriveMotor) # Replace 0 with MotorID 
   SetDriveMotorsToClosedLoopControl()
   MoveDriveMotorsIncremental(AutomaticMovingDistance) 
   Tolerance = 0.5
   
   # The following Code checks whether the vehicle moved and if it is within the Tolerance value of error
   # The code will put the drive Motors to Idle
   # TODO : This code will NOT get into idle mode here. The code for putting into idle mode has
   # to be called outside of this loop. There is not enough time for the motor to move for it to reach the final position
   # TODO check for error conditions in the motors after this call. This is a must and call the 
   #  ActOnError() function
   '''if (abs((AutomaticMovingDistance+StartingPosition)-MotorPositionEstimate)<=Tolerance):
      SetDriveMotorsToIdle()
      SetSteeringMotorsToIdle()
      # TODO Erroneous place to add this variable, will not be set if the Vehicle did not Move
      StartingPosition=AutomaticMovingDistance+StartingPosition
   else:
      print("Vehicle is Moving")
   '''

   #Readerror()
   # TODO If there is no Error (check if there is an error before updating this

'''
def ExecuteUserInput():
  while GPIO.input(MovePin)==GPIO.LOW: # Run forever
    if GPIO.input(DirectionPin)==GPIO.LOW:
      print("DirectionChange")
      #MoveVehicle(-500)
    else:
      print("button pressed")
      #MoveVehicle(500)
'''

def DebugPrint(DebugString):
    if (DEBUG == True):
        print(DebugString)




def disconnect():
   import can
   import threading

   while True:
      if not bus.is_connected:
          print("CANbus connection lost! Program terminated.")
            # Insert code here to kill program
          raise SystemExit
      else:
            # Wait for a short time before checking connection again
          threading.Event().wait(1)


   check_can_thread = threading.Thread(target=check_can_connection)
   check_can_thread.start()




def ReadAxisErrorStatus(): #TODO Rewritten as ReadAxisStatus
     global bus 
     message = bus.recv()
     #print(message)
     '''
     if message.arbitration_id == 0x029:
        rxd1 = "{:02x}".format(message.data[0])
        rxd2 = "{:02x}".format(message.data[1])
        rxd3 = "{:02x}".format(message.data[2])
        rxd4 = "{:02x}".format(message.data[3])
        rxd5 = "{:02x}".format(message.data[4])
        rxd6 = "{:02x}".format(message.data[5])
        rxd7 = "{:02x}".format(message.data[6])
        rxd8 = "{:02x}".format(message.data[7])
        
        rxdata1 = str(rxd1) + str(rxd2) + str(rxd3) + str(rxd4)
        rxdata2 = str(rxd5) + str(rxd6) + str(rxd7) + str(rxd8)
        
        fdata = struct.unpack('<f', binascii.unhexlify(rxdata1.replace(' ', '')))[0]
        position = fdata
        fdata = struct.unpack('<f', binascii.unhexlify(rxdata2.replace(' ', '')))[0]
        velocity = fdata

        print('position : ' + str(position))
        print('velocity : ' + str(velocity))

     elif message.arbitration_id == 0x021:a
     '''
     if message.arbitration_id == 0x021:
         axis_err   = ((message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8)
                       | (message.data[0]))
         cur_state  = message.data[4]
         con_status = message.data[7]
         printDebug(str(axis_err) + '  ' + str(cur_state) + '  ' + str(con_status))
         printDebug(message)




#ARBITRATIONID = AXISID << 5 | COMMANDID

#######################################  ReadStatus AxisErrors, EncoderErrors, ClearErrors RebootOdrive ###
def ReadCanIDfromHeartbeatMsg(CanBusID,MaxCount):
    global bus 
    counter = 0 
    hbeat = 0 
    eEstimate = 0
    Unknown = 0
    while True : 
       message = bus.recv()
      
       print(message)
       #print ("Message.arbitration_id: {:02x}".format(message.arbitration_id) )

       ReceivedMotorID = (message.arbitration_id & 0x0E0 ) >> 5
       CMDID = message.arbitration_id  & 0x1F
       print(" MotorID {:02x} CMDID : {:02X}".format(ReceivedMotorID,CMDID)) 
       if (CMDID == 0x01) : hbeat = hbeat +1 # print("hearbeat msg")
       elif (CMDID == 0x09) : eEstimate = eEstimate + 1 #print ("Encoder Estimate")
       else : Unknown += 1 # print ("Unknown CMDID : {:02x}".format(CMDID))

       if (CMDID == 0x01) : # its the heart beat that produced at start
           print("ReceivedMotorID : {:02x} CMDID : {:02X}".format(ReceivedMotorID,CMDID))



       if (ReceivedMotorID == CanBusID):
           print("Received Heartbeat Msg from CanBusID :{:02x} ".format(CanBusID))    
           return True
       
       if (ReceivedMotorID != CanBusID):
           print("canbus not reciving ")
           counter = counter+1
           print("counter :", counter)     
       if (counter > MaxCount) :
           print(" NO message Received even after Heartbeat Msg from CanBusID : :{:02x} ".format(CanBusID))    
           print( "HeartBeat : {} , eEstimate :{} , Unknown : {} counter : {}".format(hbeat, eEstimate, Unknown, counter) )
           return False

def ReadAxisStatusFromHeartbeatMsg(MotorId):  
    global bus 
    # Considering ID as 1
    # Axis Error and reading Heartbeat message
    ArbitrationID = MotorId << 5 | 0x1 # 0x1 is Heartbeat Message ID
    while True :
     message = bus.recv()
     #print(message)
     print ("Message.arbitration_id: {}".format(message.arbitration_id) )
     print ("ArbitrationID : {}".format(ArbitrationID) )

     ReceivedMotorID = (message.arbitration_id >> 5 ) 
     print("ReceivedMotorID : {}".format(ReceivedMotorID)) 
     #if (MotorId == ReceivedMotorID) : break  
     if (message.arbitration_id == ArbitrationID):
         axis_err   = ((message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8)
                       | (message.data[0]))
         cur_state  = message.data[4]
         con_status = message.data[7]
         print(str(axis_err) + '  ' + str(cur_state) + '  ' + str(con_status))
         print(message)
         break
def ReadMotorErrors():
 ReadMotorError(0)  
 ReadMotorError(1)  
 ReadMotorError(2)  
 ReadMotorError(3)  

def RebootAllOdrives():
  RebootOdrive(0)
  RebootOdrive(1)

def ResetOdrives():
 RebootOdrive(SteeringMotorLeft )
 RebootOdrive(SteeringMotorRight)
 RebootOdrive(SteeringMotorFront )
 RebootOdrive(FrontDriveMotor )
 RebootOdrive(LeftBackDriveMotor )
 RebootOdrive(RightBackDriveMotor )


def ReadMotorError(MotorId):  
    global bus 
    # Send this message from the controller to get Motor Error 
    ArbitrationID = MotorId << 5 | 0x3 # 0x3 is Reading Motor Error
    message = can.Message(arbitration_id = ArbitrationID , is_remote_frame = True, is_extended_id = False)
    bus.send(message)


    # Put this code to read Motor Error 
    while True :
     message = bus.recv()

    
     ReceivedMotorID = (message.arbitration_id & 0x0E0 ) >> 5
     CMDID = message.arbitration_id  & 0x1F 
     if (CMDID == 0x03) : print("received ReadMotorError Respone")

     if message.arbitration_id == ArbitrationID :
         motor_err   = ((message.data[3] << 56) | (message.data[3] << 48) | (message.data[3] << 40) | (message.data[3] << 32) 
            | (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | (message.data[0]))
         print(str(motor_err))
         print(message)
         break
#Endof def ReadMotorError(MotorId):  

def WaitForMotorToStabilise(MotorId,Position):
   #TODO read the status of the motorId and check whether it has reached the position or timedOut
   # Return success if it is reached or Error if it timeout
   time.sleep(10)

     
def TestSteeringMotor(MotorId):
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   SetMotorToClosedLoop(MotorId)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   MoveMotorPositionAbsolute(MotorId,5)
   WaitForMotorToStabilise(MotorId,5)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   MoveMotorPositionAbsolute(MotorId,0)
   WaitForMotorToStabilise(MotorId,0)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   MoveMotorPositionAbsolute(MotorId,-5)
   WaitForMotorToStabilise(MotorId,-5)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   MoveMotorPositionAbsolute(MotorId,0)
   WaitForMotorToStabilise(MotorId,0)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))
   SetMotorToIdle(MotorId)
   Error = ReadAxisError(MotorId)
   if( Error !=0 ) :
       print("Error in Motor : {} {}".format(MotorId, Error))

# TODO : This code is not tested    
def TestDriveMotors():
   print("VehicleCanBusInterface.TestDriveMotors: WARNING, CODE NOT TESTED")
   MotorList = [FrontDriveMotor, RightBackDriveMotor, LeftBackDriveMotor]
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 
   for MotorId in MotorList :
      SetMotorToClosedLoop(MotorId)
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error)
 
   #MoveDriveMotorsToSpeed(0)
   MoyyveDriveMotorsIncremental(0)
   WaitForMotorToStabilise(0,0) # TODO Dummy parameters passed
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 

   #MoveDriveMotorsToSpeed(3)
   MoveDriveMotorsIncremental(200) #incremental distance in mm
   WaitForMotorToStabilise(3,0) # TODO Dummy parameters passed
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 

   #MoveDriveMotorsToSpeed(-3)
   MoveDriveMotorsIncremental(-200) #incremental distance in mm
   WaitForMotorToStabilise(0,0) # TODO Dummy parameters passed
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 

   #MopveDriveMotorsToSpeed(0)
   MoveDriveMotorsIncremental(0) #incremental distance in mm
   WaitForMotorToStabilise(0,0) # TODO Dummy parameters passed
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 

   for MotorId in MotorList :
      SetMotorToIdle(MotorId)
   for MotorId in MotorList :
      Error = ReadAxisError(MotorId )
      if Error : return (Error) 

   return(0) 
#EndOf def TestDriveMotors
    
def ReadEncoderError(MotorId):  
    global bus 
    # Send this message from the controller to get Encoder Error 
    ArbitrationID = MotorId << 5 | 0x4 # 0x4 is Reading ENcoder Error
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)

    # Put this code to read Encoder Error
    while True :
     message = bus.recv()
     if message.arbitration_id == ArbitrationID :
         encoder_err   = ((message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | (message.data[0]))
         print(str(encoder_err))
         print(message)
         break 

def test_Encoder_posEst():

    global bus 
    # Send this message from the controller to get EncoderPostionEstimate 
    ArbitrationID = MotorId << 5 | 0x009 # 0x9 is Reading Encoder Position Estimate
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)

    # Put this code to read Encoder Error
    while True :
     message = bus.recv()
     if message.arbitration_id == ArbitrationID :
         encoder_pos_est   = ((message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | (message.data[0]))
         print(str(encoder_pos_est))
         print(message)
         break 

# Reboot ODrive
def RebootOdrive(MotorId):
    global bus 
    #message = can.Message(arbitration_id = 0x036, is_remote_frame = True, is_extended_id = False)
    ArbitrationID = MotorId << 5 | 0x016 # 0x6 Reboot Code 
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)

def ClearAxisErrors(MotorId):
    global bus 
    # Clear Errors
    #message = can.Message(arbitration_id = 0x038, is_remote_frame = True, is_extended_id = False)
    ArbitrationID = MotorId << 5 | 0x018 # 0x6 ClearError 
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)
######################################  Read Errors ###

def SetMotorToIdle(MotorId):
  global bus 
  global db
  #print("\nRequesting AXIS_STATE_IDLE axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Axis_State')
  data = msg.encode({'Axis_Requested_State': 0x01}) # Idle 
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")
    return (False)
  return(True)

## End  ofSetMotorToIdle

## End  ofSetMotorToIdle


def SetMotorToClosedLoop(MotorId):
  global bus 
  global db
  #print("\nRequesting AXIS_STATE_CLOSE_LOOP_CONTROL (0x0x8) on axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Axis_State')
  data = msg.encode({'Axis_Requested_State': 0x08}) # ClosedLoop 
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can is working first")
    return (False)
  return(True)
'''
## End SetMotorControlToVelocityMode
def SetMotorControlToVelocityMode(MotorId):
  global bus 
  global db
  print("\nRequesting ) on axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Controller_Mode')
  #msg = db.get_message_by_name('Set_Controller_Mode')
  data = msg.encode({'Axis_Requested_State': 0x08}) # ClosedLoop 
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can is working first")
    return (False)
  return(True)
'''
def SetDriveMotorsToPositionControlMode():
  SetMotorToPositionControlMode(LeftBackDriveMotor)
  SetMotorToPositionControlMode(FrontDriveMotor)
  SetMotorToPositionControlMode(RightBackDriveMotor)


def SetDriveMotorsToTorqueControlMode():
  SetMotorToTorqueControlMode(LeftBackDriveMotor)
  SetMotorToTorqueControlMode(FrontDriveMotor)
  SetMotorToTorqueControlMode(RightBackDriveMotor)

def SetDriveMotorsToVelocityControlMode():
  SetMotorToVelocityControlMode(LeftBackDriveMotor)
  SetMotorToVelocityControlMode(FrontDriveMotor)
  SetMotorToVelocityControlMode(RightBackDriveMotor)
 


def SetMotorToPositionControlMode(MotorId):
  global bus
  global db
  #print("\nRequesting change of control mode in axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Controller_Mode')
  data = db.encode_message('Set_Controller_Mode', {'Control_Mode': 0X03, 'Input_Mode': 0X05})
  #controlMode:ControlmodePositionControl   inputMode:Inputmode_Trap_Traj
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")
    return (False)
  return(True)

def SetMotorToVelocityControlMode(MotorId):
  global bus 
  global db
  #print("\nRequesting change of control mode in axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Controller_Mode')
  data = db.encode_message('Set_Controller_Mode', {'Control_Mode': 0X02, 'Input_Mode': 0X02}) 
  #controlMode:ControlmodeVelocityControl   inputMode:Inputmode_Vel_ramp
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")
    return (False)
  return(True)



def SetMotorToTorqueControlMode(MotorId):
  global bus 
  global db
  print("\nRequesting change of control mode in axisID: " + str(MotorId))
  msg = db.get_message_by_name('Set_Controller_Mode')
  data = db.encode_message('Set_Controller_Mode', {'Control_Mode': 0X01, 'Input_Mode': 0X06})
  #controlMode:ControlmodeTorqueControl   inputMode:Inputmode_Torque_Ramp
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("Message NOT sent!  Please verify can0 is working first")
    return (False)
  return(True)





def MoveMotorToSpeed(MotorID, MotorRPS):
    #message = can.Message(arbitration_id = 0x0D| MotorID << 5, data = [int(txdata1,16),int(txdata2,16),int(txdata3,16),
    #                                                      int(txdata4,16)], is_extended_id = False)
    #msg = db.get_message_by_name('Set_Input_Vel')
    #msg = db.get_message_by_name('Set_Input_Pos')
    #data = db.encode_message('Set_Input_Vel', {'Input_Vel': MotorRPS})
    data = db.encode_message('Set_Input_Vel', {'Input_Vel': MotorRPS, 'Input_Torque_FF':0.0})
    print(f" MoveMotorToSpeed Input Velocity Data : {data}\n")
    #data = db.encode_message('Set_Input_Torque', {'Input_Torque':TorqueValue})
    #data = msg.encode({'Axis_Requested_State': 0x08}) # ClosedLoop 
    message = can.Message(arbitration_id = 0x0D| MotorID << 5, is_extended_id=False, data=data)
    #msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)

    try:
      bus.send(message)
      #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
       print("CanBusError in SetMotorToVelocity Message NOT sent!  Please verify can is working first")
       return (False)

    return(True)

def BuggyMoveMotorToSpeed(MotorID, MotorRPS):
    Inputdata = canlib.IEEE754(MotorRPS)
    print(f"Inputdata {Inputdata}") 
    txdata1 = str(Inputdata[6]) + str(Inputdata[7])
    txdata2 = str(Inputdata[4]) + str(Inputdata[5])
    txdata3 = str(Inputdata[2]) + str(Inputdata[3])
    txdata4 = str(Inputdata[0]) + str(Inputdata[1])
    print(f"Data {Inputtxdata1 , txdata2, txdata2, txdata4}\n")
    message = can.Message(arbitration_id = 0x0D| MotorID << 5, data = [int(txdata1,16),int(txdata2,16),int(txdata3,16),
                                                          int(txdata4,16)], is_extended_id = False)
    try:
      bus.send(message)
      #print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
       print("CanBusError in SetMotorToVelocity Message NOT sent!  Please verify can is working first")
       return (False)

    return(True)

                                                                
def RunMotorAtTorque(MotorId,TorqueValue):
  global bus 
  global db
  print("\n Running Motor : " + str(MotorId) + "At Torque :{}".format(TorqueValue))
  msg = db.get_message_by_name('Set_Input_Torque')
  data = db.encode_message('Set_Input_Torque', {'Input_Torque':TorqueValue})
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)
 
  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("RunMotorAtTorque >> Message NOT sent!  Please verify can is working first")
    return (False)
  return(True)
 
def MoveMotorPositionAbsolute(MotorId,Position): # Position is no of motor rotations
  global bus
  #print("\n Moving Motor : " + str(MotorId) + "To Position :{}", format(Position))
  if (DEBUGINFO ) : StartTime = time.time()
  # Commenting out ReadAxis Error , Heartbeat ReadAxis is taking more than 90 milliseconds now 

  errorCode = ReadAxisError(MotorId)
  if( errorCode != 0) :
     print("VehicleCanBusInterface.MoveMotorPositionAbsolute:: Error in Motor :{}".format(MotorId))
     print("VehicleCanBusInterface.MoveMotorPositionAbsolute:: Error Value :{}".format(errorCode))
     print("VehicleCanBusInterface.MoveMotorPositionAbsolute:: Will Not be able to Move motor")
     print("VehicleCanBusInterface.MoveMotorPositionAbsolute:: Exiting the Program")
     # Reset the Odrives 
     ResetOdrives()
     exit()
  msg = db.get_message_by_name('Set_Input_Pos')
  data = db.encode_message('Set_Input_Pos', {'Input_Pos':Position, 'Vel_FF':0.0, 'Torque_FF':0.0})
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("MoveMotorPositionAbsolute >>Message NOT sent!  Please verify can is working first")
    return (False)
  if (DEBUGINFO ) : printDEBUGINFO( "Time For : MoveMotorPositionAbsolute {}".format(1000*(time.time() - StartTime ))) 
  return(True)
#End Of def MoveMotorPositionAbsolute

def SetTrapTrajVelLimit(MotorId,VelocityLimit): # Position is no of motor rotations
  global bus
  #print("\n Moving Motor : " + str(MotorId) + "To Velocity :{}", format(VelocityLimit))
  msg = db.get_message_by_name('Set_Traj_Vel_Limit')
  data = db.encode_message('Set_Traj_Vel_Limit', {'Traj_Vel_Limit':VelocityLimit})
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("MoveMotorPositionAbsolute >>Message NOT sent!  Please verify can is working first")
    return (False)
  return(True)

def SetCurrentVelocityLimits(MotorId,CurrentLimit,VelocityLimit): # Position is no of motor rotations
  global bus
  #print("\n Moving Motor : " + str(MotorId) + "To Velocity :{}", format(VelocityLimit))
  msg = db.get_message_by_name('Set_Limits')
  data = db.encode_message('Set_Limits', {'Current_Limit':CurrentLimit,'Velocity_Limit':VelocityLimit})
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("MoveMotorPositionAbsolute >>Message NOT sent!  Please verify can is working first")
    return (False)
  return(True)


def SetDriveMotorCurrentAndVelocityLimits(CurrentLimit,VelocityLimit):
   SetCurrentVelocityLimits(LeftBackDriveMotor,CurrentLimit,VelocityLimit)
   SetCurrentVelocityLimits(FrontDriveMotor,CurrentLimit,VelocityLimit)
   SetCurrentVelocityLimits(RightBackDriveMotor,CurrentLimit,VelocityLimit)

def SetSteeringMotorCurrentAndVelocityLimits(CurrentLimit,VelocityLimit):
   SetCurrentVelocityLimits(SteeringMotorLeft,CurrentLimit,VelocityLimit)
   SetCurrentVelocityLimits(SteeringMotorRight,CurrentLimit,VelocityLimit)
   SetCurrentVelocityLimits(SteeringMotorFront,CurrentLimit,VelocityLimit)

def SetDriveMotorTrapTrajVelLimit(Vel_Limit):
   if (not SetTrapTrajVelLimit(LeftBackDriveMotor,Vel_Limit) ):
     print("SetDriveMotorTrapTrajVelLimit : LeftBackDriveMotorFailled to SetTrapTrajVelLimit") 
     exit()
   if (not SetTrapTrajVelLimit(FrontDriveMotor,Vel_Limit)) :
     print("SetDriveMotorTrapTrajVelLimit : FrontDriveMotorto SetTrapTrajVelLimit") 
     print("Failled to SetTrapTrajVelLimit")
     exit()
   if (not SetTrapTrajVelLimit(RightBackDriveMotor,Vel_Limit)) :
     print("SetDriveMotorTrapTrajVelLimit : RightBackDriveMotorto SetTrapTrajVelLimit") 
     print("Failled to SetTrapTrajVelLimit") 
     exit()
#Endof def SetDriveMotorTrapTrajVelLimit

def SetSteeringMotorTrapTrajVelLimit(Vel_Limit):
   if(not SetTrapTrajVelLimit(SteeringMotorLeft,Vel_Limit)) :
     print("SetSteeringMotorTrapTrajVelLimit SteeringMotorLeft Failled to SetTrapTrajVelLimit") 
     print(": FrontDriveMotorto SetTrapTrajVelLimit") 
     exit()
   if(not SetTrapTrajVelLimit(SteeringMotorRight,Vel_Limit)) :
     print("SetSteeringMotorTrapTrajVelLimit SteeringMotorLeft Failled to SetTrapTrajVelLimit") 
     print("Failled to SetTrapTrajVelLimit") 
     exit()
   if(not SetTrapTrajVelLimit(SteeringMotorFront,Vel_Limit)):
     print("SetSteeringMotorTrapTrajVelLimit SteeringMotorFront Failled to SetTrapTrajVelLimit") 
     exit()
    
def SetTrapTrajAccelDecelLimits(MotorId,DecelLimit,AccelLimit): # Position is no of motor rotations
  global bus
  msg = db.get_message_by_name('Set_Traj_Accel_Limits')
  data = db.encode_message('Set_Traj_Accel_Limits', {'Traj_Decel_Limit':DecelLimit,'Traj_Accel_Limit':AccelLimit})
  msg = can.Message(arbitration_id=msg.frame_id | MotorId << 5, is_extended_id=False, data=data)
  #print(db.decode_message('Set_Axis_State', msg.data))
  #print(msg)

  try:
    bus.send(msg)
    #print("Message sent on {}".format(bus.channel_info))
  except can.CanError:
    print("MoveMotorPositionAbsolute >>Message NOT sent!  Please verify can is working first")
    exit()
    return (False)
  return(True)

def SetDriveMotorTrapTrajAccelDecelLimits(AccelLimit,DecelLimit):
   SetTrapTrajAccelDecelLimits(LeftBackDriveMotor,AccelLimit,DecelLimit)
   SetTrapTrajAccelDecelLimits(FrontDriveMotor,AccelLimit,DecelLimit)
   SetTrapTrajAccelDecelLimits(RightBackDriveMotor,AccelLimit,DecelLimit)

def SetSteeringMotorTrapTrajAccelDecelLimits(AccelLimit,DecelLimit):
   SetTrapTrajAccelDecelLimits(SteeringMotorLeft,AccelLimit,DecelLimit)
   SetTrapTrajAccelDecelLimits(SteeringMotorRight,AccelLimit,DecelLimit)
   SetTrapTrajAccelDecelLimits(SteeringMotorFront,AccelLimit,DecelLimit)
#end of def SetSteeringMotorTrapTrajAccelDecelLimits(AccelLimit,DecelLimit):
 
# ReadAxisError read the heartbeat msg and check for error 
# If there are no heartbeat msg from the controller then it will return -1 
# TODO  ReadAxis Error using the Heartbeat is taking more than 90 milliseconds now 
# TODO  Need to implement a fast mechanism to read the errors
def ReadAxisError(AxisID):
  if (DEBUGINFO ) : StartTime = time.time()
  global bus 
  global msg
  '''
  if (AxisID not in ActiveMotorList) :
       print(f"This Motor {AxisID} is not in ActiveMotorList :{ActiveMotorList}\n")
       return (0)
  '''

  msg = bus.recv()
  for msg in bus:
    if msg.arbitration_id == ((AxisID << 5) | db.get_message_by_name('Heartbeat').frame_id):
        errorCode = db.decode_message('Heartbeat', msg.data)['Axis_Error']
        if errorCode == 0x00:
            printDebug("No errors")
        else:
            print("Axis error!  Error code: "+str(hex(errorCode)))
        if (DEBUGINFO ) : printDEBUGINFO( "Time For ReadAxisError :  {}".format(1000*(time.time() - StartTime ))) 
        return(errorCode)
  print("VehicleCanBusInterface.ReadAxisError() :: Not Able to read the Status for AxisId :  {}".format(AxisId))
  if (DEBUGINFO ) : printDEBUGINFO( "Not able to Read :: Time For ReadAxisError :  {}".format(1000*(time.time() - StartTime ))) 
  return(-1)
#End of def ReadAxisError(AxisID):

# ReadAxisState will read heartbeats and check for status
# Will return -1 if there are no heartbeat msgs received for the particular axis
# TODO : Not tested to working 
def ReadAxisState(AxisID):
  print("WARNING VehicleCanBusInterface.ReadAxisState() :: Code not yet tested")
  msg = bus.recv()
  for msg in bus:
    if msg.arbitration_id == ((AxisID << 5) | db.get_message_by_name('Heartbeat').frame_id):
        Axis_Error = db.decode_message('Heartbeat', msg.data)['Axis_Error']
        Controller_Flags = db.decode_message('Heartbeat', msg.data)['Controller_Flags']
        Encoder_Flags = db.decode_message('Heartbeat', msg.data)['Encoder_Flags']
        Motor_Flags = db.decode_message('Heartbeat', msg.data)['Motor_Flags']
        Axis_State = db.decode_message('Heartbeat', msg.data)['Axis_State']
        print("Axis_Error : {}", format(Axis_Error))
        print("Motor_Flags: {}", format(Motor_Flags))
        print("Encoder_Flags : {}", format(Encoder_Flags ))
        print("Controller_Flags : {}", format(Controller_Flags ))
        print("Axis_State : {}", format(Axis_State ))
        break
  print("VehicleCanBusInterface.ReadAxisState() :: Not Able to read the error")
  return(-1)

def Readerror():
   if (ReadAxisError(FrontDriveMotor) or 
      ReadAxisError(SteeringMotorFront ) ) :
      # or ReadAxisError(FrontDriveMotor) or
      # ReadAxisError(RightBackDriveMotor)) :
      return True
   else: return False 

def SetSteeringMotorsToIdle():
   SetMotorToIdle(SteeringMotorRight )
   SetMotorToIdle(SteeringMotorLeft)
   SetMotorToIdle(SteeringMotorFront)
   time.sleep(0.1)
  
def SetSteeringMotorsToClosedLoopControl():
   SetMotorToClosedLoop(SteeringMotorRight )
   SetMotorToClosedLoop(SteeringMotorLeft)
   SetMotorToClosedLoop(SteeringMotorFront)
   time.sleep(0.1)


def SetDriveMotorsToIdle():
   SetMotorToIdle(LeftBackDriveMotor)
   SetMotorToIdle(FrontDriveMotor)
   SetMotorToIdle(RightBackDriveMotor)
   time.sleep(0.1)
 
def SetDriveMotorsToClosedLoopControl():
   SetMotorToClosedLoop(LeftBackDriveMotor)
   SetMotorToClosedLoop(FrontDriveMotor)
   SetMotorToClosedLoop(RightBackDriveMotor)
   time.sleep(0.1)

def SetVehicleToIdle():
   SetDriveMotorsToIdle()
   SetSteeringMotorsToIdle()

def Move3WheelSteeringMotorsTo(AbsoluteRotation): #Absolute Rotation
   # AbsoluteRotation should be less 0.5 and greater than -05
   # the negative sign in the front wheel steering motor is for the three wheel steering.
   MoveMotorPositionAbsolute(SteeringMotorLeft ,AbsoluteRotation)
   MoveMotorPositionAbsolute(SteeringMotorRight ,AbsoluteRotation)
   MoveMotorPositionAbsolute(SteeringMotorFront ,-AbsoluteRotation)
# Endof def Move3WheelSteeringMotorsTo

# Pivoting of the Vehicle with LeftSteeringmotor to 135, RightSteeringMotor to 45 and MidSteeringWheel to 90 
# This code is not tested yet
def Pivot3WheelSteeringMotorsTo():
   MoveMotorPositionAbsolute(SteeringMotorLeft , 135 )
   MoveMotorPositionAbsolute(SteeringMotorRight ,45)
   MoveMotorPositionAbsolute(SteeringMotorFront ,-90)
# Endof def Pivot3WheelSteeringMotorsTo 



def MoveSteeringMotorsTo(AbsoluteRotation): #Absolute Rotation
   # In Manual Mode only the Front Steering Motor is steered.
    # AbsoluteRotation should be less 0.5 and greater than -05
   # the negative sign in the front wheel steering motor is for the three wheel steering.
   MoveMotorPositionAbsolute(SteeringMotorLeft ,AbsoluteRotation)
   MoveMotorPositionAbsolute(SteeringMotorRight ,AbsoluteRotation)
#   MoveMotorPositionAbsolute(SteeringMotorFront ,-AbsoluteRotation)
# Endof def MoveSteeringMotorsTo(

# to integrate the row to row movement on 30 Mar 2023
def MoveSteeringMotorsTo90degreerotationLeft(): #Absolute Rotation
   # In this Manual Mode Left   the Steering Motor is steered to 90 degree.
    # AbsoluteRotation should be less 0.5 and greater than -05
   SetSteeringMotorsToClosedLoopControl()
   MoveMotorPositionAbsolute(SteeringMotorLeft ,12.5)
   MoveMotorPositionAbsolute(SteeringMotorRight ,12.5)
   MoveMotorPositionAbsolute(SteeringMotorFront ,12.5)
# Endof def MoveSteeringMotorsTo90degreerotationLeft

def MoveSteeringMotorsTo90degreerotationRight(): #Absolute Rotation
   # In this Manual Mode Right the Steering Motor is steered to 90 degree.
   SetSteeringMotorsToClosedLoopControl()
    # AbsoluteRotation should be less 0.5 and greater than -05
   MoveMotorPositionAbsolute(SteeringMotorLeft ,-12.5)
   MoveMotorPositionAbsolute(SteeringMotorRight ,-12.5)
   MoveMotorPositionAbsolute(SteeringMotorFront ,-12.5)
# Endof def MoveSteeringMotorsTo90degreerotationRight

def MoveSteeringMotorsTo0degreerotation(): #Absolute Rotation
   # In Manual Mode only the Front Steering Motor is steered.
    # AbsoluteRotation should be less 0.5 and greater than -05
   SetSteeringMotorsToClosedLoopControl()
   MoveMotorPositionAbsolute(SteeringMotorLeft ,0)
   MoveMotorPositionAbsolute(SteeringMotorRight ,0)
   MoveMotorPositionAbsolute(SteeringMotorFront ,0)
# Endof def MoveSteeringMotorsTo0degreerotation

AbsoluteRotationFor135Degrees = (135/360) * GearRatioSteeringMotor 
AbsoluteRotationFor90Degrees  = (90/360) * GearRatioSteeringMotor 
AbsoluteRotationFor45Degrees  = (45/360) * GearRatioSteeringMotor 

def SetVehicleToPivot(PivotDirection):
  SetSteeringMotorsToClosedLoopControl()
  if (PivotDirection == PIVOT_LEFT):
     MoveMotorPositionAbsolute(SteeringMotorLeft , -AbsoluteRotationFor135Degrees )
     MoveMotorPositionAbsolute(SteeringMotorRight, -AbsoluteRotationFor45Degrees)
     MoveMotorPositionAbsolute(SteeringMotorFront,  AbsoluteRotationFor90Degrees)
  elif (PivotDirection == PIVOT_RIGHT):
     MoveMotorPositionAbsolute(SteeringMotorLeft ,  AbsoluteRotationFor45Degrees )
     MoveMotorPositionAbsolute(SteeringMotorRight,  AbsoluteRotationFor135Degrees)
     MoveMotorPositionAbsolute(SteeringMotorFront, -AbsoluteRotationFor90Degrees)
  elif (PivotDirection == PIVOT_NONE):
     MoveMotorPositionAbsolute(SteeringMotorLeft ,0)
     MoveMotorPositionAbsolute(SteeringMotorRight,0)
     MoveMotorPositionAbsolute(SteeringMotorFront,0)
#END of def SetVehicleToPivot():

 
def AutomaticMoveSteeringMotorsTo(AbsoluteRotation): #Absolute Rotation
    # AbsoluteRotation should be less 0.5 and greater than -05
   MoveMotorPositionAbsolute(SteeringMotorLeft ,AbsoluteRotation)
   MoveMotorPositionAbsolute(SteeringMotorRight ,AbsoluteRotation)
   MoveMotorPositionAbsolute(SteeringMotorFront ,AbsoluteRotation)
 
def AkermannsSteeringMotorsTo(AbsoluteRotation): #Absolute Rotation
   import math

   wheel_base=1500
   wheel_tread=1800
   wheel_center_distance=wheel_tread/2

   if (AbsoluteRotation ==0):
     MoveMotorPositionAbsolute(SteeringMotorLeft ,AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorRight ,AbsoluteRotation)
   else:
     angle_of_rotation= (AbsoluteRotation*360)/GearRatioSteeringMotor
     radius_of_curvature=abs(wheel_base/(math.tan(math.radians(angle_of_rotation))))
     outside_wheel_angle=math.degrees(math.atan(wheel_base/(radius_of_curvature+wheel_center_distance)))
     inside_wheel_angle=math.degrees(math.atan(wheel_base/abs((radius_of_curvature-wheel_center_distance))))
     print(angle_of_rotation)
     print(radius_of_curvature)
     print(outside_wheel_angle)
     print(inside_wheel_angle)

     akermansrotation_outside_AbsoluteRotation=(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=(inside_wheel_angle/360)* GearRatioSteeringMotor

   if (AbsoluteRotation>0):
     akermansrotation_outside_AbsoluteRotation=(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=(inside_wheel_angle/360)* GearRatioSteeringMotor
     MoveMotorPositionAbsolute(SteeringMotorRight ,akermansrotation_outside_AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorLeft ,akermansrotation_inside_AbsoluteRotation)
     print(akermansrotation_outside_AbsoluteRotation)
     print(akermansrotation_inside_AbsoluteRotation)

   
   if (AbsoluteRotation<0):
     akermansrotation_outside_AbsoluteRotation=-(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=-(inside_wheel_angle/360)* GearRatioSteeringMotor
     MoveMotorPositionAbsolute(SteeringMotorLeft ,akermansrotation_outside_AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorRight ,akermansrotation_inside_AbsoluteRotation) 
     print(akermansrotation_outside_AbsoluteRotation)
     print(akermansrotation_inside_AbsoluteRotation)

def ThreeWheelAkermannsSteeringMotorsTo(AbsoluteRotation): #Absolute Rotation
   import math

   wheel_base=1500
   wheel_tread=1800
   wheel_center_distance=wheel_tread/2

   if (AbsoluteRotation ==0):
     MoveMotorPositionAbsolute(SteeringMotorLeft ,AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorRight ,AbsoluteRotation)
   else:
     angle_of_rotation= (AbsoluteRotation*360)/GearRatioSteeringMotor
     radius_of_curvature=abs(wheel_base/(math.tan(math.radians(angle_of_rotation))))
     outside_wheel_angle=math.degrees(math.atan(wheel_base/(radius_of_curvature+wheel_center_distance)))
     inside_wheel_angle=math.degrees(math.atan(wheel_base/abs((radius_of_curvature-wheel_center_distance))))
     rear_wheel_angle = (outside_wheel_angle + inside_wheel_angle)/2
     print(angle_of_rotation)
     print(radius_of_curvature)
     print(outside_wheel_angle)
     print(inside_wheel_angle)
     print(rear_wheel_angle)

     akermansrotation_outside_AbsoluteRotation=(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=(inside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_rear_AbsoluteRotation=(rear_wheel_angle/360)* GearRatioSteeringMotor

   if (AbsoluteRotation>0):
     akermansrotation_outside_AbsoluteRotation=(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=(inside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_rear_AbsoluteRotation=-(rear_wheel_angle/360)* GearRatioSteeringMotor
     MoveMotorPositionAbsolute(SteeringMotorRight ,akermansrotation_outside_AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorLeft ,akermansrotation_inside_AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorFront ,akermansrotation_rear_AbsoluteRotation)
     print(akermansrotation_outside_AbsoluteRotation)
     print(akermansrotation_inside_AbsoluteRotation)
     print(akermansrotation_rear_AbsoluteRotation)

   
   if (AbsoluteRotation<0):
     akermansrotation_outside_AbsoluteRotation=-(outside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_inside_AbsoluteRotation=-(inside_wheel_angle/360)* GearRatioSteeringMotor
     akermansrotation_rear_AbsoluteRotation=(rear_wheel_angle/360)* GearRatioSteeringMotor
     MoveMotorPositionAbsolute(SteeringMotorLeft ,akermansrotation_outside_AbsoluteRotation)
     MoveMotorPositionAbsolute(SteeringMotorRight ,akermansrotation_inside_AbsoluteRotation) 
     MoveMotorPositionAbsolute(SteeringMotorFront ,akermansrotation_rear_AbsoluteRotation)
     print(akermansrotation_outside_AbsoluteRotation)
     print(akermansrotation_inside_AbsoluteRotation)
     print(akermansrotation_rear_AbsoluteRotation)



def MoveDriveMotorsToSpeed( MotorRPS): # MOVE MOTORS TO SPEED VALUE MotorRPS
   MoveMotorToSpeed(LeftBackDriveMotor , MotorRPS)
   MoveMotorToSpeed(RightBackDriveMotor, MotorRPS)
   MoveMotorToSpeed(FrontDriveMotor    , MotorRPS)
def MoveDriveMotorsToTorque( MotorTorque): # MOVE MOTORS TO Torque VALUE MotorTorque
   RunMotorAtTorque(LeftBackDriveMotor , MotorTorque)
   RunMotorAtTorque(RightBackDriveMotor, MotorTorque)
   RunMotorAtTorque(FrontDriveMotor    , MotorTorque)

def MoveDriveMotorsIncremental(IncrementalDistance) :
   global bus

   #SetDriveMotorsToPositionControlMode()
   #SetDriveMotorsToClosedLoopControl()

   if (DEBUGINFO ) : StartTime = time.time()
   #TODO the following task to  be done for each motors  
   MotorPositionEstimateLeftBack, CurrentVelocity = OdriveCanBusHelperFunctions.GetPositionAndVelocity(bus, LeftBackDriveMotor) # Replace 0 with MotorID 
   MotorPositionEstimateRightBack, CurrentVelocity = OdriveCanBusHelperFunctions.GetPositionAndVelocity(bus, RightBackDriveMotor) # Replace 0 with MotorID 
   MotorPositionEstimateFront, CurrentVelocity = OdriveCanBusHelperFunctions.GetPositionAndVelocity(bus, FrontDriveMotor) # Replace 0 with MotorID 
   DistanceMoving = IncrementalDistance 
   print("MotorPositionEstimateLeftBack:",MotorPositionEstimateLeftBack)
   print("MotorPositionEstimateRightBack:",MotorPositionEstimateRightBack)
   print("MotorPositionEstimateFront:",MotorPositionEstimateFront)

   LeftBackDriveMotorRotation = (DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateLeftBack 
   RightBackDriveMotorRotation =(DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateRightBack
   FrontDriveMotorRotation = (DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateFront

   MoveMotorPositionAbsolute(RightBackDriveMotor,RightBackDriveMotorRotation)
   MoveMotorPositionAbsolute(FrontDriveMotor,FrontDriveMotorRotation)
   MoveMotorPositionAbsolute(LeftBackDriveMotor,LeftBackDriveMotorRotation)

   print("MotorPositionEstimateLeftBack:",MotorPositionEstimateLeftBack)
   print("LeftBackDriveMotorRotation :",LeftBackDriveMotorRotation )
   if (DEBUGINFO ) : printDEBUGINFO( "Time For MoveDriveMotorsIncremental :  {}".format(1000*(time.time() - StartTime ))) 
#ENd Of def MoveDriveMotorsIncremental(IncrementalDistance) :

#TODO Remove this code or correct it to work for all conditions
def MoveDriveMotorsTo(AbsolutePosition): # AbsolutePosition is in millimeters
   DistanceMoving = AbsolutePosition  # Why do we do this? 

   #LeftFrontDriveMotorRotation = GearRatioLeftFrontDriveMotor*(DistanceMoving/WheelDistancePerRotation)
   LeftBackDriveMotorRotation = (DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateLeftBack 
   RightBackDriveMotorRotation =(DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateRightBack
   FrontDriveMotorRotation = (DistanceMoving/WheelDistancePerRotationOfMotor) + MotorPositionEstimateFront

   #MoveMotorPositionAbsolute(LeftFrontDriveMotor,LeftFrontDriveMotorRotation)
   #MoveMotorPositionAbsolute(RightFrontDriveMotor,RightFrontDriveMotorRotation)
   #MoveMotorPositionAbsolute(RightBackDriveMotor,RightBackDriveMotorRotation)
   MoveMotorPositionAbsolute(LeftBackDriveMotor,LeftBackDriveMotorRotation)
   MoveMotorPositionAbsolute(RightBackDriveMotor,RightBackDriveMotorRotation)
   MoveMotorPositionAbsolute(FrontDriveMotor,FrontDriveMotorRotation)


# To stop the vehicle the Drive Motors are kept at '0' velocity
def   StopVehicle():
   SetDriveMotorsToClosedLoopControl()
   SetSteeringMotorsToClosedLoopControl()
   MoveDriveMotorsToSpeed(0)
   # Not doing any changes to the Vehicle Steering
     
def MoveVehicleBySpeedAndAngle(VehicleRequiredSpeed, VehicleSteeringWheelRotation):
 # VehicleRequiredSpeed in meters/hour , 
 # VehicleSteeringWheelRotation between 0 and 1 (360degrees)
   global CurrentPos
   global TimeForUnitDistance
   global CurrentSteerPosition
   global TimeForFullWheelSteeringRotation 
   # DestinationPosition = IncrementalDistance + CurrentPos 

   IncrementalAngle =  abs(VehicleSteeringWheelRotation- CurrentSteerPosition)
   # Value should be between +/- MaxSteeringAngle/360
   # +0.25 rotation goes to +90 degrees  
   MotorRotationAngle = VehicleSteeringWheelRotation * GearRatioSteeringMotor
   #Readerror()
   SpeedInMMperSecond = VehicleRequiredSpeed * 1000 /3600 # Speed conversion in millimeter per second
   WheelRotationsPerSecond = SpeedInMMperSecond / WheelDistancePerRotation 
   MotorRPS = WheelRotationsPerSecond * GearRatioFrontDriveMotor 

   SetDriveMotorsToClosedLoopControl()
   SetSteeringMotorsToClosedLoopControl()
   print(f" Vehicle Motors Moving at RPS : {MotorRPS} and Angle :{MotorRotationAngle}\n")
   MoveDriveMotorsToSpeed( MotorRPS)
   MoveSteeringMotorsTo(MotorRotationAngle)
#   AkermannsSteeringMotorsTo(MotorRotationAngle)

   #TODO : replace this by reading Position of the wheel
   #SteeringWaitTime = (IncrementalAngle * TimeForFullWheelSteeringRotation) 
   #SleepTime = DrivingWaitTime
   #time.sleep(SleepTime)
   #print(f"** Sleeping for {SleepTime}")
   #SetSteeringMotorsToIdle()
   #Readerror()
   #time.sleep(11)

   #SetDriveMotorsToIdle()
   #SetSteeringMotorsToIdle()
   #Readerror()
   # TODO If there is no Error (check if there is an error before updating this
   #CurrentPos = DestinationPosition
   CurrentSteerPosition = VehicleSteeringWheelRotation # Update the current SteeringPosition

# End of MoveVehicleByVelocityAndAngle 
def MoveVehicleBySpeed(VehicleRequiredSpeed):
 # VehicleRequiredSpeed in meters/hour , 
   global CurrentPos
   global TimeForUnitDistance
   global CurrentSteerPosition
   global TimeForFullWheelSteeringRotation 
   # DestinationPosition = IncrementalDistance + CurrentPos 

   IncrementalAngle =  abs(VehicleSteeringWheelRotation- CurrentSteerPosition)
   # Value should be between +/- MaxSteeringAngle/360
   # +0.25 rotation goes to +90 degrees  
   MotorRotationAngle = VehicleSteeringWheelRotation * GearRatioSteeringMotor
   #Readerror()
   SpeedInMMperSecond = VehicleRequiredSpeed * 1000 /3600 # Speed conversion in millimeter per second
   WheelRotationsPerSecond = SpeedInMMperSecond / WheelDistancePerRotation 
   MotorRPS = WheelRotationsPerSecond * GearRatioFrontDriveMotor 

   SetDriveMotorsToClosedLoopControl()
   print(f" Vehicle Motors Moving at RPS : {MotorRPS} \n")
   MoveDriveMotorsToSpeed( MotorRPS)


# End of MoveVehicleByVelocity 

def MoveVehicleByTorqueAndAngle(VehicleRequiredTorque, VehicleSteeringWheelRotation):
 # VehicleRequiredSpeed in meters/hour , 
 # VehicleSteeringWheelRotation between 0 and 1 (360degrees)
   global CurrentPos
   global TimeForUnitDistance
   global CurrentSteerPosition
   global TimeForFullWheelSteeringRotation 
   # DestinationPosition = IncrementalDistance + CurrentPos 

   IncrementalAngle =  abs(VehicleSteeringWheelRotation- CurrentSteerPosition)
   # Value should be between +/- MaxSteeringAngle/360
   # +0.25 rotation goes to +90 degrees  
   MotorRotationAngle = VehicleSteeringWheelRotation * GearRatioSteeringMotor
   #Readerror()
   MotorTorque=VehicleRequiredTorque
   SetDriveMotorsToClosedLoopControl()
   SetSteeringMotorsToClosedLoopControl()
   print(f" Vehicle Motors Moving at Torque : {MotorTorque} and Angle :{MotorRotationAngle}\n")
   MoveDriveMotorsToTorque( MotorTorque)
   AkermannsSteeringMotorsTo(MotorRotationAngle)
#   MoveSteeringMotorsTo(MotorRotationAngle)

   #TODO : replace this by reading Position of the wheel
   #SteeringWaitTime = (IncrementalAngle * TimeForFullWheelSteeringRotation) 
   #SleepTime = DrivingWaitTime
   #time.sleep(SleepTime)
   #print(f"** Sleeping for {SleepTime}")
   #SetSteeringMotorsToIdle()
   #Readerror()
   #time.sleep(11)

   #SetDriveMotorsToIdle()
   #SetSteeringMotorsToIdle()
   #Readerror()
   # TODO If there is no Error (check if there is an error before updating this
   #CurrentPos = DestinationPosition
   CurrentSteerPosition = VehicleSteeringWheelRotation # Update the current SteeringPosition

# End of MoveVehicleByVelocityAndAngle 

def MoveVehicleByTorque(VehicleRequiredTorque):
   global CurrentPos
   global TimeForUnitDistance

   #Readerror()
   MotorTorque=VehicleRequiredTorque
   SetDriveMotorsToClosedLoopControl()
   print(f" Vehicle Motors Moving at Torque : {MotorTorque} \n")
   MoveDriveMotorsToTorque( MotorTorque)
'''
   if (DEBUGINFO ) : StartTime = time.time()
   if (DEBUGINFO ) : printDEBUGINFO( "Time For MoveVehicle:  {}".format(1000*(time.time() - StartTime ))) 
'''
def MoveVehicle(IncrementalDistance, AbsoluteRotation): # Incremental Distance is in Millimeters
   global CurrentPos
   global TimeForUnitDistance
   global CurrentSteerPosition
   global TimeForFullWheelSteeringRotation 
   if (DEBUGINFO ) : StartTime = time.time()
   IncrementalAngle =  abs(AbsoluteRotation - CurrentSteerPosition)
   # -0.25 goes to -90 degrees 
   # +0.25 rotation goes to +90 degrees  
   MotorRotationAngle = AbsoluteRotation * GearRatioSteeringMotor

#   SetDriveMotorsToClosedLoopControl()
#   SetSteeringMotorsToClosedLoopControl()
   # Replacing the Torque mode with Position Control Mode 
#   MoveSteeringMotorsTo(MotorRotationAngle)
   #Move3WheelSteeringMotorsTo(MotorRotationAngle)
   MoveDriveMotorsIncremental(IncrementalDistance) 
   AkermannsSteeringMotorsTo(MotorRotationAngle)
#   MoveSteeringMotorsTo(MotorRotationAngle)
   '''
   #TODO : replace this by reading Position of the wheel
   DrivingWaitTime = abs(IncrementalDistance) * TimeForUnitDistance 
   SteeringWaitTime = (IncrementalAngle * TimeForFullWheelSteeringRotation) 
   if (SteeringWaitTime > DrivingWaitTime ): SleepTime = SteeringWaitTime 
   else : SleepTime = DrivingWaitTime
   time.sleep(SleepTime)
   print(f"** Sleeping for {SleepTime}")
   #SetSteeringMotorsToIdle()
   #Readerror()
   #time.sleep(11)

   SetDriveMotorsToIdle()
   SetSteeringMotorsToIdle()
   '''
   #Readerror()
   # TODO If there is no Error (check if there is an error before updating this
   CurrentSteerPosition = AbsoluteRotation # Update the current SteeringPosition
   if (DEBUGINFO ) : printDEBUGINFO( "Time For MoveVehicle:  {}".format(1000*(time.time() - StartTime ))) 

# End of Move Vehicle ()


def SteerVehicle(AbsoluteRotation): # Floating point Number from -0.25 to +0.25
   global CurrentSteerPosition
   global TimeForFullWheelSteeringRotation 
   IncrementalAngle =  abs(AbsoluteRotation - CurrentSteerPosition)
   # -0.25 goes to -90 degrees 
   # +0.25 rotation goes to +90 degrees  
   if (abs(AbsoluteRotation) <= 0.25) :
        MotorRotationAngle = AbsoluteRotation * GearRatioSteeringMotor
        #Readerror()
        #SetSteeringMotorsToClosedLoopControl()
        MoveSteeringMotorsTo(MotorRotationAngle )
        WaitTime = (IncrementalAngle * TimeForFullWheelSteeringRotation) 
        print(f"** Sleeping for {WaitTime}")
        # TODO change this CheckForCompletionStatus???
        time.sleep( WaitTime) 
        #SetSteeringMotorsToIdle()
        CurrentSteerPosition = AbsoluteRotation # Update the current SteeringPosition
        #Readerror()
   else : 
        print("Error in Rotation Angle given\n") 
        print("Exiting Execution \n") 
        exit()

   # TODO If there is no Error (check if there is an error before updating this

# End of Move Vehicle ()


def TestMoveWorking():
   print("Setting Motor 0 to Idle")
   time.sleep(5)
   SetMotorToIdle(0)
   #SetMotorToIdle(1)
   SetMotorToIdle(2)
   SetMotorToIdle(4)
   # ReadAxisError(0)
   print("Setting Motor 0 to ClosedLoop")
   time.sleep(5)
   SetMotorToClosedLoop(0)
   #SetMotorToClosedLoop(1)
   SetMotorToClosedLoop(2)
   SetMotorToClosedLoop(4)
  
   # ReadAxisError(0)
   print(" MovingMotor 0 by 0.2")
   time.sleep(5)
   MoveMotorPositionAbsolute(0, -100)
   #MoveMotorPositionAbsolute(1, 2)
   MoveMotorPositionAbsolute(2, -100)
   MoveMotorPositionAbsolute(4, -100)

   # ReadAxisError(0)
   print("Setting Motor 0 to Idle")
   time.sleep(5)
   SetMotorToIdle(0)
   #SetMotorToIdle(1)
   SetMotorToIdle(2)
   SetMotorToIdle(4)

   #ReadAxisError(0)
   #MoveVehicle(1)

# TODO : Not tested to work
def ReadMotorStatus():
   #ReadAxisState(LeftFrontDriveMotor)
   ReadAxisState(LeftBackDriveMotor)
   ReadAxisState(RightFrontDriveMotor)
   ReadAxisState(RightBackDriveMotor)

def TestReboot():
    # Reboot Odrive, then MoveMotors, This should not work, since motors would have gone to Idle state
   RebootOdrive(0)
   RebootOdrive(1)
   RebootOdrive(2)
   RebootOdrive(3)
   RebootOdrive(4)
   RebootOdrive(5)

def Initialise():
   GPIOInitialize()
   CanBusInitialize()
   RebootOdrive(0)
   RebootOdrive(1)
   RebootOdrive(2)
   RebootOdrive(3)
   RebootOdrive(4)
   RebootOdrive(5)
#   SteerVehicle(0.0)
#TODO Test the code? Use the code in VehicleControl.Execute as the first line before doing any other actions
def check_can_connection(bus):
    import can
    import threading
   # import odrive
    #import bus
    while True:
        print(bus.state)
        if bus.state != can.BusState.ACTIVE:
            print("CANbus connection lost! Program terminated.")
            return False
        else:
            print("its Fine")
        #else:
            # Wait for a short time before checking connection again
            threading.Event().wait(1)
        #if (bus.state == can.BusState.ERROR):
           # print("it failed")


    check_can_thread = threading.Thread(target=check_can_connection)
    check_can_thread.start()



if __name__== "__main__" :
   #RebootOdrive(LeftFrontDriveMotor) # Reboots the Left Odrive
   #RebootOdrive(RightFrontDriveMotor) # Reboots the Right Odrive
   #ReadAxisStatusFromHeartbeatMsg(LeftFrontDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(LeftBackDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(RightFrontDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(RightBackDriveMotor)
   #Readerror() # Read Error Status before running
   #TestReboot()
   #Buttonstart()
   Initialise()
   #:Readerror() # Check Error status after Completing
   while True:
      print( "Waiting for user inputs ")
      ExecuteUserInput()
      #break 

