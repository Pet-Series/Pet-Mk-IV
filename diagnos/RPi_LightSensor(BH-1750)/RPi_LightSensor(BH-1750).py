# Sample Python3 App for BH1750FVI I2C Digital Light Intensity Sensor
# Sites I used...
# http://www.raspberrypi-spy.co.uk/2015/03/bh1750fvi-i2c-digital-light-intensity-sensor/
# http://www.raspberrypi-spy.co.uk/2014/11/enabling-the-i2c-interface-on-the-raspberry-pi/
#
# +----- +---------------+------+
# |Module| Desc	GPIO     |      |
# |PCB   | Header        |Pins  |
# +----- +---------------+------+
# |GND 	 | Ground	       |P1-06 |
# |ADD	 | Address select|P1-06 |
# |SDA	 | I2C SDA	      |P1-03 |
# |SCL	 | I2C SCL	      |P1-05 |
# |VCC	 | 3.3V	         |P1-01 |
# +----- +---------------+------+
#
## Kolla vilka i2c enheter som är inkopplade just nu 
## $ sudo i2cdetect -y 1
##     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
##00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
##10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##20: -- -- -- 23 -- -- -- -- -- -- -- -- -- -- -- -- 
##30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
##70: -- -- -- -- -- -- -- -- 

import smbus
import time
 
# Define some constants from the datasheet

# If the ADDR pin to ground the address used by the device is 0x23.
# If the ADDR pin is tied to 3.3V the address is 0x5C. 
DEVICE     = 0x23 # Default device I2C address
 
POWER_DOWN = 0x00 # No active state
POWER_ON   = 0x01 # Power on
RESET      = 0x07 # Reset data register value

 
# Start measurement at 4lx resolution. Time typically 16ms.
CONTINUOUS_LOW_RES_MODE = 0x13
# Start measurement at 1lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_1 = 0x10
# Start measurement at 0.5lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_1 = 0x20
# Start measurement at 0.5lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_2 = 0x21
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_LOW_RES_MODE = 0x23
 
#bus = smbus.SMBus(0) # Rev 1 Pi uses 0
bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
 
def convertToNumber(data):
  # Simple function to convert 2 bytes of data
  # into a decimal number
  return ((data[1] + (256 * data[0])) / 1.2)
 
def readLight(addr=DEVICE):
  data = bus.read_i2c_block_data(addr,ONE_TIME_HIGH_RES_MODE_1)
  return convertToNumber(data)
 
def main():
 
  # The while loop keeps taking readings every 200ms until you press CTRL-C
  while True:
    print ("Light Level : " + str(readLight()) + " lx")
    time.sleep(0.5)
   
if __name__=="__main__":
   main()
