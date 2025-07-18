
# TODO Move these to a Global File called PragatiConfig
# Pin Definition for different Functions of Vehicle 
Global DirectionSwitch   =  21  # 1 forward, 0 reverse direction move in Manual  
Global StopButton        =  4   # Stops the vehicle, should act as an electric brake 
Global AutoManualSwitch  =  26  # Enable is Auto, Disabled is Manual
Global ArmStart          =  16  # Used in Manual Mode to start Arm operation PressButton

 List the active Set of Motors connected
# Software will hang if the motors are listed here but not connected
#ActiveMotorList = [SteeringMotorLeft,SteeringMotorRight,
#                  SteeringMotorFront,FrontDriveMotor 
#                  LeftBackDriveMotor ,RightBackDriveMotor ]
ActiveMotorList = [SteeringMotorLeft,LeftBackDriveMotor ]

WheelDiameter = 20 * 25.4  # in millimeters for a Wheel Diameter of 20 inch 
WheelDistancePerRotation = 3.141519 * WheelDiameter


