#!/usr/bin/python
# Sample Python3 App for HC-SR04 Ultrasonic distance
# Sites I used...
# http://www.raspberrypi-spy.co.uk/2012/12/ultrasonic-distance-measurement-using-python-part-1
# http://www.raspberrypi-spy.co.uk/2013/01/ultrasonic-distance-measurement-using-python-part-2
#
# +-------+--------------+------+
# |Module | Desc GPIO    |      |
# |PCB    | Header       |Pins  |
# +-------+--------------+------+
# |GROUND | Ground	      |P1-06 |
# |TRIGGER| GPIO23       |P1-16 |
# |ECHO   | GPIO24       |P1-18 |
# |       | Conv. 5V->3V |      |
# |VCC	  | 5V	         |P1-02 |
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

# -----------------------
# Define some functions
# -----------------------

def measure():
  # This function measures a distance
  GPIO.output(GPIO_TRIGGER, True)
  time.sleep(0.00001)
  GPIO.output(GPIO_TRIGGER, False)
  pulsStart = time.time()

  while GPIO.input(GPIO_ECHO)==0:
    pulsStart = time.time()

  while GPIO.input(GPIO_ECHO)==1:
    pulsStop = time.time()

  pulsDuration = pulsStop - pulsStart # Calculate pulse duration
  distance = (pulsDuration * 34300)/2

  return distance

def measure_average():
  # This function takes 3 measurements and
  # returns the average.
  distance1=measure()
  time.sleep(0.1)
  distance2=measure()
  time.sleep(0.1)
  distance3=measure()
  distance = distance1 + distance2 + distance3
  distance = distance / 3
  return distance

# -----------------------
# Main Script
# -----------------------

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
# To Avoid - "RuntimeWarning: This channel is already in use, continuing anyway."
GPIO.setwarnings(False)

# Define GPIO to use on Pi
GPIO_TRIGGER = 23
GPIO_ECHO    = 24

print ("Ultrasonic Measurement by HC-SR04")

# Set pins as output and input
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGGER, False)

# Wrap main content in a try block so we can
# catch the user pressing CTRL-C and run the
# GPIO cleanup function. This will also prevent
# the user seeing lots of unnecessary error
# messages.
try:
  print ("Try...")
  while True:
    distance = measure_average()
    print ('Distance : %.1f' % distance )
    time.sleep(1)

except KeyboardInterrupt:
  # User pressed CTRL-C
  # Reset GPIO settings
  print ("Cancel! Ctrl-C pressed...")
  GPIO.cleanup()
