#!/usr/bin/python
# Sample Python3 app for reading 3x Optical Sensors
#
# +-------+--------------+------+
# |Module | Desc GPIO    |      |
# |       | Header       |Pins  |
# +-------+--------------+------+
# |SensorL| GPIO17	     |P1-11 |
# |SensorC| GPIO27       |P1-13 |
# |SensorR| GPIO22       |P1-15 |
# |       |              |      |
# |VCC	  | 3.3V	     |      |
# |GND	  | GND	         |      |
# +-------+--------------+------+

##???????????????????????????????????????????
## GPIO.wait_for_edge(GPIO_ECHO, GPIO.BOTH)
## start = time.time()
## GPIO.wait_for_edge(GPIO_ECHO, GPIO.BOTH)
## stop = time.time()
##
## and initialized with
## GPIO.add_event_detect(GPIO_ECHO, GPIO.BOTH)
##
##R educing CPU load significantly.
##I  tried using GPIO.RISING the GPIO.FALLING
##???????????????????????????????????????????

# -----------------------
# Import required Python libraries
# -----------------------
import time
import RPi.GPIO as GPIO
from sense_hat import SenseHat

# -----------------------
# Define some functions
# -----------------------

# N/A

# -----------------------
# Main Script
# -----------------------

# Use BCM GPIO references, instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

sense = SenseHat()
sense.clear()

# Define GPIO to use on Pi
GPIO_opticalLeft = 17
GPIO_opticalCenter = 27
GPIO_opticalRight = 22


# Define basic coulors
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)


print ("Optical sensors")

# Set pins as output and input
GPIO.setup(GPIO_opticalLeft  ,GPIO.IN)
GPIO.setup(GPIO_opticalCenter,GPIO.IN)
GPIO.setup(GPIO_opticalRight ,GPIO.IN)

# Wrap main content in a try block so we can
# catch the user pressing CTRL-C and run the
# GPIO cleanup function. This will also prevent
# the user seeing lots of unnecessary error
# messages.
try:
  print ("Try...")
  while True:
    opticalLeft = GPIO.input(GPIO_opticalLeft)
    opticalCenter = GPIO.input(GPIO_opticalCenter)
    opticalRight = GPIO.input(GPIO_opticalRight)
    
    print ('Left: %s  Center: %s  Right: %s' % (opticalLeft, opticalCenter, opticalRight ))
    

    # examples using (x, y, pixel) on the SenseHAT
    if opticalLeft:
        sense.set_pixel(0, 0, red)
    else:
        sense.set_pixel(0, 0, green)
    
    if opticalCenter:
        sense.set_pixel(1, 0, red)
    else:
        sense.set_pixel(1, 0, green)
    
    if opticalRight:
        sense.set_pixel(2, 0, red)
    else:
        sense.set_pixel(2, 0, green)
    
    time.sleep(1)

except KeyboardInterrupt:
    # User pressed CTRL-C
    # Reset GPIO settings
    print ("Cancel! Ctrl-C pressed...")
    GPIO.cleanup()
    sense.clear()

print ("När syns detta???")