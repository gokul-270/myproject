#import os
import can
import numpy
import binascii
import struct
import time

def ReadAxisStatusFromCANHeartBeat(bus, CanID):
  HeartBeatCMD = 0x01
  TimeoutError = 0xDEAD
  axis_err = 0 
  while True :
      message= bus.recv(timeout=1.0) # Timeout in seconds
      if message is None:
          print('Timeout occurred, no message.')
          return (TimeoutError)
      else:
          # Not sure whether the previous message will be lost because of the following line
          for message in bus:
              # first check the received CanID & if the message is heartbeat message
              ReceivedCanID = (message.arbitration_id & (0x3E0)) >> 5 
              ReceivedCMDID = message.arbitration_id & (0x01F)
              if ( (ReceivedCanID == CanID) and (ReceivedCMDID == HeartbeatCMD)):
                   axis_err   = ((message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8)
                                 | (message.data[0]))
                   cur_state  = message.data[4]
                   con_status = message.data[7]
                   print(str(axis_err) + '  ' + str(cur_state) + '  ' + str(con_status))
                   print(message)
                   return (axis_err) 



def GetPositionAndVelocity(bus, CanID) :
 # CanID is the canid of the motor where the position and velocity is required
 # bus is the canbus handler
 # Send a comand for reading the position and velocity estimate
 CMDPositionVelocity = 0x09
 ArbitrationID = CanID << 5 | CMDPositionVelocity 
 message = can.Message(arbitration_id = ArbitrationID , is_remote_frame = True, is_extended_id = False)
 bus.send(message)

 while True:
     message = bus.recv()
     print(message)
     ReceivedCanID = (message.arbitration_id & (0x3E0)) >> 5 
     ReceivedCMDID = message.arbitration_id & (0x01F)
     print( "ReceivedCanID ", ReceivedCanID )
     print( "ReceivedCMDID ", ReceivedCMDID  )
     
     if ( (message.dlc > 7) and (ReceivedCMDID == CMDPositionVelocity ) and (ReceivedCanID == CanID)) :
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

        print('position : ' + str((position)))
        print('velocity : ' + str((velocity)))
        return( float(position), float(velocity))    

