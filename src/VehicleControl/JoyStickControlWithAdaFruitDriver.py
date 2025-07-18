# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain
import time
import RPi.GPIO as GPIO
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008



CANEnable = 8 # GPIO8 Chip Enable CE0 (SPIO)
ADCEnable = 7 # GPIO7 Chip Enable CE1 (SPIO)
def Initialise():
# All pins are referred to BCM reference such as .1 i
   GPIO.setmode(GPIO.BCM)
   #GPIO.setup(CANEnable, GPIO.OUT, initial=GPIO.HIGH)
   #GPIO.output(CANEnable, GPIO.HIGH)
   GPIO.setwarnings(False)

# This code uses the Software SPI drives, the hardware SPI is not working now
# TODO: If the HW SPI works we will switch to the HW SPI driver
# Software SPI configuration:
'''
CLK  = 11
MISO = 9
MOSI = 10
CS   = 7
'''

# Remapping the CLK, MISO, MOSI and CS to the following pins
CLK  = 21
MISO =  20
MOSI = 19
CS   = 7

mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Hardware SPI configuration:
# SPI_PORT   = 0
# SPI_DEVICE = 0
# mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

XChannel = 0
YChannel = 1

def ReadJoyStickXY():
  # First Disable CANbus
  #GPIO.output(CANEnable, GPIO.HIGH)
  # Enable ADC 
  GPIO.output(ADCEnable, GPIO.LOW)
  XValue = mcp.read_adc(XChannel)
  YValue = mcp.read_adc(YChannel)
  GPIO.output(ADCEnable, GPIO.HIGH)
  #GPIO.output(CANEnable, GPIO.LOW)
  return (XValue, YValue)

def demoCode():  
 print('Reading MCP3008 values, press Ctrl-C to quit...')
 # Print nice channel column headers.
 print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
 print('-' * 57)
 # Main program loop.
 while True:
    GPIO.output(8, GPIO.HIGH)
    # Read all the ADC channel values in a list.
    values = [0]*8
    for i in range(8):
        # The read_adc function will get the value of the specified channel (0-7).
        values[i] = mcp.read_adc(i)
    # Print the ADC values.
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    # Pause for half a second.
    time.sleep(0.5)

if __name__== "__main__" :
 while True:
   Initialise()
   print("Reading X and Y Values of Joystick")
   XValue , YValue = ReadJoyStickXY()
   print(f"XValue: {XValue}, YValue: {YValue}\n")
   time.sleep(1)
