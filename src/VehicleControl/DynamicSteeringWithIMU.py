#!/usr/bin/python3

#@author by saikiran
    #for the stop and go mechanism for the vehicle on 350W 36V hub motor
import time

'''
odrv0.config.brake_resistance = 0.5
odrv0.config.enable_brake_resistor = True

odrv0.axis0.motor.config.resistance_calib_max_voltage = 5
odrv0.axis0.motor.config.pole_pairs=15
odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.axis0.motor.config.torque_constant = 8.27 /  9.01

odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
odrv0.axis0.encoder.config.cpr = 90
odrv0.axis0.encoder.config.calib_scan_distance = 150
odrv0.axis0.encoder.config.ignore_illegal_hall_state = True

odrv0.config.gpio9_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio10_mode = GPIO_MODE_DIGITAL
odrv0.config.gpio11_mode = GPIO_MODE_DIGITAL

odrv0.axis0.encoder.config.bandwidth = 100
#odrv0.axis0.encoder.config.bandwidth = 1000
odrv0.axis0.controller.config.pos_gain = 1
#odrv0.axis0.controller.config.pos_gain = 20
odrv0.axis0.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
# odrv0.axis0.controller.config.vel_gain = 0.1666666716337204
odrv0.axis0.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
#odrv0.axis0.controller.config.vel_integrator_gain = 0.3333333432674408
odrv0.axis0.controller.config.vel_limit = 10
#odrv0.axis0.controller.config.vel_limit = 2.0
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
'''

from ast import For
import odrive
import time
#from tkinter import W

'''
    SteeringAngleErrorTolerance = 3 # float(3*50/360)
    max_steer_limit = 45 # float(45*50/360)
    min_steer_limit = -45 #-float(45*50/360)
    min_steer_limit = 3 #float(3*50/360)
    steering_gear_ratio = float(50/360)
    req_yaw = int(0)
    #current_yaw = float(4.5)
    MotorRotation = float()
   
    if(current_yaw > 180): #IMU_gives_output_yaw_in_(0,2pi)_converting_it_to_(-pi,pi)
        current_yaw = current_yaw - 360
        
    CorrectionAngle = (req_yaw - current_yaw) 
    MotorRotation = CorrectionAngle * steering_gear_ratio
  
    if( CorrectionAngle > max_steer_limit):
        MotorRotation = max_steer_limit * steering_gear_ratio   
    elif (CorrectionAngle < min_steer_limit):
        MotorRotation = min_steer_limit * steering_gear_ratio   
    elif (abs(CorrectionAngle) < SteeringAngleErrorTolerance) :
        MotorRotation = 0

    return (MotorRotation)
'''
# Converts Yaw angle in degrees to NoOfMotorRotations , a floating point value
# This considers the gear ration in use
# ASSUMPTIONS : At power up it is assumed that the wheel is at zero yaw angle
# TODO : How to correct situations where wheel is not aligned to zero yaw angle at Powerup 
def ConvertYawToMotorRotation( current_yaw ):
    max_steer_limit = 45 # in Degreees
    neg_max_steer_limit = -45  # in Degrees
    # The min_steer_limit is derived from a 5 cm error from the straigth line 
    # 
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
    MotorRotaion = correction_angle * steeringGearRatioinRotation 
    return(MotorRotaion)
#def InitializeSystem()
i = int(1) #signaal_from_MARC_or_RESET_SWITCH
d = int(0) #direction_of_vehicle
PragatiOdrive = odrive.find_any()
Odriveid0 = PragatiOdrive.odrv0
Odriveid1 = PragatiOdrive.odrv1
Motorid0 = axis0
Motorid1 = axis1

#Odriveid1 = "odrv1"
#Odriveid2 = "odrv0"
Motorid0 = "axis0"
Motorid1 = "axis1"
set_pos = int(0)
# TODO : Fine tune the gear ratio of the drive motor
Gear_ratio = int(3) #motor_gear_ratio_is_3
# TODO : Document on how req_pos ?
req_pos = float(0.512*Gear_ratio) #defining_the_req_motor_pos_wrt_wheel_pos_considering_gear_ratio
forward_final_pos = float(req_pos)
reverse_final_pos = float(-req_pos)
if(d==0):
    FINSTRING = str(forward_final_pos) #forward_direction
elif(d==1):
    FINSTRING = str(reverse_final_pos) #reverse_direction
#POSTRING = str(set_pos)
import serial
#def InitializeIMU():
ser = serial.Serial('/dev/ttyACM0' , 9600, timeout = 1)
ser.reset_input_buffer()

def Python3Controlmode(Odriveid0, Odriveid1, Odriveid2, Motorid0, Motorid1): #closed_loop_control_for_all_motors
     #import time
     #CLOSEDLOOP0 = Odriveid0+"."+Motorid0+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #CLOSEDLOOP1 = Odriveid0+"."+Motorid1+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #CLOSEDLOOP2 = Odriveid1+"."+Motorid0+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #CLOSEDLOOP3 = Odriveid1+"."+Motorid1+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     CLOSEDLOOP4 = Odriveid2+"."+Motorid0+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     CLOSEDLOOP5 = Odriveid2+"."+Motorid1+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #exec(CLOSEDLOOP0)
     #exec(CLOSEDLOOP1)
     #exec(CLOSEDLOOP2)
     #exec(CLOSEDLOOP3)
     exec(CLOSEDLOOP4)
     exec(CLOSEDLOOP5)
 
def Controlmode(Odriveid0, Odriveid1, Odriveid2, Motorid0, Motorid1): #closed_loop_control_for_all_motors
     #import time
     Odriveid0.Motorid0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
     #CLOSEDLOOP1 = Odriveid0+"."+Motorid1+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #CLOSEDLOOP2 = Odriveid1+"."+Motorid0+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     #CLOSEDLOOP3 = Odriveid1+"."+Motorid1+".requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL"
     Odriveid2.Motorid0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
     Odriveid2.Motorid1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

     #exec(CLOSEDLOOP0)
     #exec(CLOSEDLOOP1)
     #exec(CLOSEDLOOP2)
     #exec(CLOSEDLOOP3)
     exec(CLOSEDLOOP4)
     exec(CLOSEDLOOP5)
     print("Executing Controlmode")

#defin a Variable IMUValueInFloat so that it can store the last readValue
global IMUValueInFloat
IMUValueInFloat = 0
# ReadIMU Reads value through the serial port of Arduino 
#    IMU is mounted on Arduino and the values are pushed to the serial port in the Arduino in asynchronous manner
#    TODO will have to change this to read directly to RPI4 to eliminate the Arduino interface.
def ReadIMU():
    global IMUValueInFloat
    #ser.reset_input_buffer()
    line =""
    time.sleep(0.2)
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
    return(IMUValueInFloat)
    #return(0)
        

def SteeringMotors(Odriveid2, Motorid0, Motorid1, MotorRotation): #steering_function
    STEERSTRING0 = Odriveid2+"."+Motorid0+"controller.input_pos = "+MotorRotation
    STEERSTRING1 = Odriveid2+"."+Motorid1+"controller.input_pos = "+MotorRotation
    exec(STEERSTRING0)
    exec(STEERSTRING1)
     

def MoveWheelsByDistance(Odriveid0, Odriveid1, Motorid0, Motorid1, FINSTRING): #moving_wheels__by_900mm
    MOVESTRING0 = Odriveid0+"."+Motorid0+".controller.move_incremental("+FINSTRING+",False)"
    MOVESTRING1 = Odriveid0+"."+Motorid1+".controller.move_incremental("+FINSTRING+",False)"
    MOVESTRING2 = Odriveid1+"."+Motorid0+".controller.move_incremental("+FINSTRING+",False)"
    MOVESTRING3 = Odriveid1+"."+Motorid1+".controller.move_incremental("+FINSTRING+",False)"
    exec(MOVESTRING0)
    exec(MOVESTRING1)
    exec(MOVESTRING2)
    exec(MOVESTRING3)
    #time.sleep(5)

def Idle(Odriveid0, Odriveid1, Odriveid2, Motorid0, Motorid1): #keeping_the_vehicle_into_IDLE
    IDLESTRING0 = Odriveid0+"."+Motorid0+".requested_state = "+"AXIS_STATE_IDLE"
    IDLESTRING1 = Odriveid0+"."+Motorid1+".requested_state = "+"AXIS_STATE_IDLE"
    IDLESTRING2 = Odriveid1+"."+Motorid0+".requested_state = "+"AXIS_STATE_IDLE"
    IDLESTRING3 = Odriveid1+"."+Motorid1+".requested_state = "+"AXIS_STATE_IDLE"
    IDLESTRING4 = Odriveid2+"."+Motorid0+".requested_state = "+"AXIS_STATE_IDLE"
    IDLESTRING5 = Odriveid2+"."+Motorid1+".requested_state = "+"AXIS_STATE_IDLE"
    print("IDLESTRING")
    exec(IDLESTRING0)
    exec(IDLESTRING1)
    exec(IDLESTRING2)
    exec(IDLESTRING3)
    exec(IDLESTRING4)
    exec(IDLESTRING5) 

def MoveWheels(): #Moving_four_wheels
    Controlmode(Odriveid0, Odriveid1, Odriveid2, Motorid0, Motorid1)
    time.sleep(1)
    #MoveVehicleByDistance(Odriveid0, Odriveid1, Motorid0, Motorid1, FINSTRING)
    #print("WHEELS_ARE_MOVING")
    #time.sleep(6)

def SteeringVehicle():
    for i in [0,1,2]:
        #ReadIMU()
        SteeringMotors(Odriveid2, Motorid0, Motorid1, MotorRotation)
        time.sleep(1)
    print("VEHICLE_STEERING")


def RunVehicle():
    MoveWheels()
    SteeringVehicle()
    #time.sleep(2)
    #Idle(Odriveid0, Odriveid1, Odriveid3, Motorid0, Motorid1)
'''    
if(i==0):
    RunVehicle() 
''' 
#if __name__ == "__main__":
while True:
        #ReadIMU()
        print ("Reading IMU")
        CurrentYawAngle = ReadIMU()
        MotorRotation = ConvertYawToMotorRotation(CurrentYawAngle)
        print(CurrentYawAngle)
        print(MotorRotation)
        RunVehicle()

'''
if __name__ == "__main__":
    while True:
        #ReadIMU()
        CurrentYawAngle = ReadIMU()
        MotorRotation = ConvertYawToMotorRotation(CurrentYawAngle)
        #print( "Steering Angle ")
       #print(MotorRotation)
        #print("CurrentYawAngle" )
        #print(CurrentYawAngle )
        time.sleep(0.02)
'''
