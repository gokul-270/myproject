from time import sleep
import serial

def unsignedToSigned(n, byte_count): 
  return int.from_bytes(n.to_bytes(byte_count, 'little', signed=False), 'little', signed=True)    

dir_value = 0
drv_value = 0
ser = serial.Serial ("/dev/ttyS0", 115200)
arr = bytearray(10)
while True:
    arr = ser.read(6)              #read serial port
    print("read Value")
    print( arr)
    sleep(0.03)
    data_left = ser.inWaiting()             #check for remaining byte
    arr += ser.read(data_left)
    drv_value = unsignedToSigned(arr[2],1)
    dir_value = unsignedToSigned(arr[3],1)
    print (drv_value)
    print (dir_value)
    print ('\n')
    

    
