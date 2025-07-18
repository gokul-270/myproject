#import gpiozero
import os
import can
import cantools
import numpy
import binascii
import struct
import time
import pigpio
#from gpiozero import Button

# Global Value Definitions
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO librarya
DirectionPin = 24
MovePin = 22


###### Software Variables 
global CurrentPos
global IncrementalPos
global TargetPos
SteeringMotorLeft = 4
SteeringMotorRight = 5 
LeftFrontDriveMotor = 1
LeftBackDriveMotor = 0
RightFrontDriveMotor = 3
RightBackDriveMotor = 2
WheelDiameter = 20 * 25.4  # Wheel Diameter of 20 inch
WheelDistancePerRotation = 3.141519 * WheelDiameter

GearRatioLeftBackDriveMotor   = -12
GearRatioLeftFrontDriveMotor  = 12
GearRatioRightFrontDriveMotor = 12
GearRatioRightBackDriveMotor  = -12

CurrentPos = 0

# End Of Global Value Definitions

def CanBusInitialize():
  os.system('sudo ip link set can0 type can bitrate 250000')
  os.system('sudo ifconfig can0 up')
  CanBusEnablePin = 8 # GPIO8 used for CANBUS enable 
  ADCEnablePin = 7 
  GPIO.setup(ADCEnablePin, GPIO.OUT, initial = GPIO.HIGH) # Disable ADC when running Canbus  
  GPIO.output(ADCEnablePin, GPIO.HIGH) # Disable ADC when running Canbus  
  db = cantools.database.load_file("odrive-cansimple.dbc")
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

# CommandID is the ID of the Command

def GPIOInitialize():
 GPIO.setwarnings(False)
 GPIO.setmode(GPIO.BCM) 
 GPIO.setup(DirectionPin , GPIO.IN, pull_up_down=GPIO.PUD_UP)
 GPIO.setup(MovePin , GPIO.IN, pull_up_down=GPIO.PUD_UP)

def ExecuteUserInput():
  while GPIO.input(MovePin)==GPIO.LOW: # Run forever
    if GPIO.input(DirectionPin)==GPIO.LOW:
      print("DirectionChange")
      MoveVehicle(-500)
    else:
      print("button pressed")
      MoveVehicle(500)

def DebugPrint(DebugString):
    if (DEBUG == True):
        print(DebugString)


def ReadAxisErrorStatus(): #TODO Rewritten as ReadAxisStatus
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
         print(str(axis_err) + '  ' + str(cur_state) + '  ' + str(con_status))
         print(message)




#ARBITRATIONID = AXISID << 5 | COMMANDID

#######################################  ReadStatus AxisErrors, EncoderErrors, ClearErrors RebootOdrive ###
def ReadCanIDfromHeartbeatMsg(CanBusID,MaxCount):
    counter = 0 
    hbeat = 0 
    eEstimate = 0
    Unknown = 0
    while True :
     message = bus.recv()
     #print(message)
     #print ("Message.arbitration_id: {:02x}".format(message.arbitration_id) )
      
     ReceivedMotorID = (message.arbitration_id & 0x0E0 ) >> 5
     CMDID = message.arbitration_id  & 0x1F 
     if (CMDID == 0x01) : hbeat = hbeat +1 # print("hearbeat msg")
     elif (CMDID == 0x09) : eEstimate = eEstimate + 1 #print ("Encoder Estimate")
     else : Unknown += 1 # print ("Unknown CMDID : {:02x}".format(CMDID))
    
     if (CMDID == 0x01) :
         print("ReceivedMotorID : {:02x} CMDID : {:02X}".format(ReceivedMotorID,CMDID)) 
     counter = counter + 1 
     #if ( ReceivedMotorID == CanBusID) :
        #print("Received Heartbeat Msg from CanBusID :{:02x} ".format(CanBusID))    
        #break 
     if (counter > MaxCount) :
        #print(" NO message Received even after Heartbeat Msg from CanBusID : :{:02x} ".format(CanBusID))    
        print( "HeartBeat : {} , eEstimate :{} , Unknown : {} counter : {}".format(hbeat, eEstimate, Unknown, counter) )
        break 


def ReadAxisStatusFromHeartbeatMsg(MotorId):  
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

def ReadMotorError(MotorId):  
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

def ReadEncoderError(MotorId):  
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

# Reboot ODrive
def RebootOdrive(MotorId):
    #message = can.Message(arbitration_id = 0x036, is_remote_frame = True, is_extended_id = False)
    ArbitrationID = MotorId << 5 | 0x016 # 0x6 Reboot Code 
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)

def ClearAxisErrors(MotorId):
    # Clear Errors
    #message = can.Message(arbitration_id = 0x038, is_remote_frame = True, is_extended_id = False)
    ArbitrationID = MotorId << 5 | 0x018 # 0x6 ClearError 
    message = can.Message(arbitration_id = ArbitrationID  , is_remote_frame = True, is_extended_id = False)
    bus.send(message)
######################################  Read Errors ###

def SetMotorToIdle(MotorId):
  print("\nRequesting AXIS_STATE_IDLE axisID: " + str(MotorId))
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

## End SetMotorToIdle
def SetMotorToClosedLoop(MotorId):
  print("\nRequesting AXIS_STATE_CLOSE_LOOP_CONTROL (0x0x8) on axisID: " + str(MotorId))
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

                                                                
def RunMotorAtTorque(MotorId,TorqueValue):
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
  print("\n Moving Motor : " + str(MotorId) + "To Position :{}", format(Position))
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
  return(True)


def ReadAxisError(AxisID):
  msg = bus.recv()
  for msg in bus:
    if msg.arbitration_id == ((AxisID << 5) | db.get_message_by_name('Heartbeat').frame_id):
        errorCode = db.decode_message('Heartbeat', msg.data)['Axis_Error']
        if errorCode == 0x00:
            print("No errors")
        else:
            print("Axis error!  Error code: "+str(hex(errorCode)))
        return(errorCode)

def ReadAxisState(AxisID):
  # msg = bus.recv()
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


def Readerror():

   if (ReadAxisError(LeftBackDriveMotor) or 
      ReadAxisError(LeftFrontDriveMotor) or
      ReadAxisError(RightFrontDriveMotor) or
      ReadAxisError(RightBackDriveMotor)) :
      return True
   else: return False 

def SetDriveMotorsToIdle():
   SetMotorToIdle(LeftFrontDriveMotor)
   SetMotorToIdle(LeftBackDriveMotor)
   SetMotorToIdle(RightFrontDriveMotor)
   SetMotorToIdle(RightBackDriveMotor)
   time.sleep(0.5)
 
def SetDriveMotorsToClosedLoopControl():
   SetMotorToClosedLoop(LeftFrontDriveMotor)
   SetMotorToClosedLoop(LeftBackDriveMotor)
   SetMotorToClosedLoop(RightFrontDriveMotor)
   SetMotorToClosedLoop(RightBackDriveMotor)
   time.sleep(1)

def RunDriveMotorsToTorque(TorqueValue):
   RunMotorAtTorque(LeftFrontDriveMotor,TorqueValue)
   RunMotorAtTorque(LeftBackDriveMotor,TorqueValue)
   RunMotorAtTorque(RightFrontDriveMotor,TorqueValue)
   RunMotorAtTorque(RightBackDriveMotor,TorqueValue)

def RunVehicleToTorque(TorqueValue,Distance):
   global CurrentPos
   TimeForUnitDistance = 3
   
   SetDriveMotorsToClosedLoopControl()

   RunDriveMotorsToTorque(TorqueValue)
   #TODO : replace this by reading Position of the wheel
   time.sleep(TimeForUnitDistance * Distance)

   SetDriveMotorsToIdle()

   # TODO If there is no Error (check if there is an error before updating this



def MoveDriveMotorsTo(AbsolutePosition): # AbsolutePosition is in millimeters
   DistanceMoving = AbsolutePosition*3.14

   LeftBackDriveMotorRotation = GearRatioLeftBackDriveMotor*(DistanceMoving/WheelDistancePerRotation)
   LeftFrontDriveMotorRotation = GearRatioLeftFrontDriveMotor*(DistanceMoving/WheelDistancePerRotation)
   RightBackDriveMotorRotation = GearRatioRightBackDriveMotor*(DistanceMoving/WheelDistancePerRotation)
   RightFrontDriveMotorRotation = GearRatioRightFrontDriveMotor*(DistanceMoving/WheelDistancePerRotation)

   MoveMotorPositionAbsolute(LeftFrontDriveMotor,LeftFrontDriveMotorRotation)
   MoveMotorPositionAbsolute(LeftBackDriveMotor,LeftBackDriveMotorRotation)
   MoveMotorPositionAbsolute(RightFrontDriveMotor,RightFrontDriveMotorRotation)
   MoveMotorPositionAbsolute(RightBackDriveMotor,RightBackDriveMotorRotation)
 
def MoveVehicle(IncrementalDistance): # Incremental Distance is in Millimeters
   global CurrentPos
   DestinationPosition = IncrementalDistance + CurrentPos 
   TimeForUnitDistance = 1  # 1/100 # permm speed of 100mm in 1second
 
   #ReadMotorStatus()

   '''
   SetDriveMotorsToIdle():
   :1'''
   Readerror()

   SetDriveMotorsToClosedLoopControl()

   MoveDriveMotorsTo( DestinationPosition)

   #TODO : replace this by reading Position of the wheel
   #time.sleep(abs(IncrementalDistance * TimeForUnitDistance) )
   time.sleep(11)

   SetDriveMotorsToIdle()
   Readerror()
   # TODO If there is no Error (check if there is an error before updating this
   CurrentPos = DestinationPosition

# End of Move Vehicle ()

def MoveWorking():
   print("Setting Motor 0 to Idle")
   time.sleep(5)
   SetMotorToIdle(0)
   SetMotorToIdle(1)
   SetMotorToIdle(2)
   SetMotorToIdle(3)
   # ReadAxisError(0)
   print("Setting Motor 0 to ClosedLoop")
   time.sleep(5)
   SetMotorToClosedLoop(0)
   SetMotorToClosedLoop(1)
   SetMotorToClosedLoop(2)
   SetMotorToClosedLoop(3)
  
   # ReadAxisError(0)
   print(" MovingMotor 0 by 0.2")
   time.sleep(5)
   MoveMotorPositionAbsolute(0, 2)
   MoveMotorPositionAbsolute(1, 2)
   MoveMotorPositionAbsolute(2, 2)
   MoveMotorPositionAbsolute(3, 2)

   # ReadAxisError(0)
   print("Setting Motor 0 to Idle")
   time.sleep(5)
   SetMotorToIdle(0)
   SetMotorToIdle(1)
   SetMotorToIdle(2)
   SetMotorToIdle(3)

   ReadAxisError(0)
   #MoveVehicle(1)

def ReadMotorStatus():
   ReadAxisState(LeftFrontDriveMotor)
   ReadAxisState(LeftBackDriveMotor)
   ReadAxisState(RightFrontDriveMotor)
   ReadAxisState(RightBackDriveMotor)

def TestReboot():
    # Reboot Odrive, then MoveMotors, This should not work, since motors would have gone to Idle state
    RebootOdrive(0)
    RebootOdrive(2)
    
if __name__== "__main__" :
   #RebootOdrive(LeftFrontDriveMotor) # Reboots the Left Odrive
   #RebootOdrive(RightFrontDriveMotor) # Reboots the Right Odrive
   #ReadAxisStatusFromHeartbeatMsg(LeftFrontDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(LeftBackDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(RightFrontDriveMotor)
   #ReadAxisStatusFromHeartbeatMsg(RightBackDriveMotor)
   #Readerror() # Read Error Status before running
   #TestReboot()
   #RebootOdrive(0)
   #RebootOdrive(2)
   GPIOInitialize()
   CanBusInitialize()
   RebootOdrive(0)
   #GPIOInitialize()
   #Buttonstart()
   #Readerror() # Check Error status after Completing
   while True:
      print( "Waiting for user inputs ")
      #ExecuteUserInput()
      #break 

