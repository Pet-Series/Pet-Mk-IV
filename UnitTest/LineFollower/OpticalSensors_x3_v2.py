#!/usr/bin/python
# Sample Python3 app for reading 3x Optical Sensors
#
# +-------+--------------+------+
# |Module | Desc GPIO    |      |
# |       | Header       |Pins  |
# +-------+--------------+------+
# |SensorL| GPIO17	 |P1-11 |
# |SensorC| GPIO27       |P1-13 |
# |SensorR| GPIO22       |P1-15 |
# |       |              |      |
# |VCC	  | 3.3V         |      |
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

# Write input value to dictionary
def opticalCallback(channel):
    if channel not in opticalDict:
        print("Weird channel from interrupt")
    else:
        opticalDict[channel] = GPIO.input(channel)

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

# Create a empty dictionary
# Is global to allow calling from within callback-function
opticalDict = {GPIO_opticalLeft: 0, GPIO_opticalCenter: 0, GPIO_opticalRight: 0}

# Set initial measurements for each channel
optocalDict[GPIO_opticalLeft] = GPIO.input(GPIO_opticalLeft)
optocalDict[GPIO_opticalCenter] = GPIO.input(GPIO_opticalCenter)
optocalDict[GPIO_opticalRight] = GPIO.input(GPIO_opticalRight)

# Tell GPIO to run a threaded callback for each change event
# https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/
GPIO.add_event_detect(GPIO_opticalLeft, GPIO.BOTH, callback=opticalCallback)
GPIO.add_event_detect(GPIO_opticalCenter, GPIO.BOTH, callback=opticalCallback)
GPIO.add_event_detect(GPIO_opticalRight, GPIO.BOTH, callback=opticalCallback)

# Wrap main content in a try block so we can
# catch the user pressing CTRL-C and run the
# GPIO cleanup function. This will also prevent
# the user seeing lots of unnecessary error
# messages.
try:
  print ("Try...")
  while True:
    
    print ('Left: %s  Center: %s  Right: %s' % (opticalDict[GPIO_opticalLeft],
                                                opticalDict[GPIO_opticalCenter],
                                                opticalDict[GPIO_opticalRight]))
    
    # examples using (x, y, pixel) on the SenseHAT
    if opticalDict[GPIO_opticalLeft]:
        sense.set_pixel(0, 0, red)
    else:
        sense.set_pixel(0, 0, green)
    
    if opticalDict[GPIO_opticalCenter]:
        sense.set_pixel(1, 0, red)
    else:
        sense.set_pixel(1, 0, green)
    
    if opticalDict[GPIO_opticalRight]:
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

