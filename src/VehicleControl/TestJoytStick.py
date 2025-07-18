
import pigpio
import time
from spidev import SpiDev
class MCP3008:
    def __init__(self, bus = 0, device = 0):
        self.bus, self.device = bus, device
        self.spi = SpiDev()
        self.open()
        self.spi.max_speed_hz = 1000000 # 1MHz
 
    def open(self):
        self.spi.open(self.bus, self.device)
        self.spi.max_speed_hz = 1000000 # 1MHz
    
    def read(self, channel = 0):
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data
            
    def close(self):
        self.spi.close()

#enable ADC this is done through pulling down CE1 signal connected to
# GPIO7
AdcEnable = 7
CanBusEnable = 8
PiGPIO = pigpio.pi()
# CAnBus and ADC are in the same Serial Port interface
# Serial Port can interface to either one of them
# To make software clean ONLY ONE of them at any point of time
PiGPIO.write(AdcEnable,1) # Enable ADC
PiGPIO.write(CanBusEnable,0) #Disable CANBUS 

adc = MCP3008()
SampleCount = 0
while True :
    SampleCount += 1
    print("SampleCount : ", SampleCount)
    Xvalue = adc.read(channel = 0)
    Yvalue = adc.read(channel = 1)
    value1 = adc.read(channel = 2)
    value2 = adc.read(channel = 2)
    value3 = adc.read(channel = 3)
    value4 = adc.read(channel = 4)
    value5 = adc.read(channel = 5)
    value6 = adc.read(channel = 6)
    value7 = adc.read(channel = 7)
    print("XValue %.2f : " %(Xvalue/1023 * 3.3)  )
    print("YValue %.2f : " %(Yvalue/1023 * 3.3)  )
    print("value1 %.2f : " %(value1/1023 * 3.3)  )
    print("value2 %.2f : " %(value2/1023 * 3.3)  )
    print("value3 : ", value3 )
    print("value4 : ", value4 )
    print("value5 : ", value5 )
    print("value6 : ", value6 )
    print("value7 : ", value7 )
    print("Change joystick")
    time.sleep (3) 
