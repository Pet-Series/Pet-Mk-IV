#!/usr/bin/python
# Python3 App for L298N - Dual H Bridge DC Stepper Motor Drive Controller Board Module
# Sites I used...
#  #
# +-----------------------------+
# |Import required Python libraries
# +-----------------------------+
from __future__ import print_function
import RPi.GPIO as GPIO
import time

#GPIO.RPI_VERSION # AttributeError: 'module' object has no attribute 'RPI_VERSION'

# +-----------------------------+
# | Use BCM GPIO references
# | (instead of physical pin numbers)
# +-----------------------------+
GPIO.setmode(GPIO.BCM)

# +-------+--------------+------+
# |Module | Desc GPIO    |RPI2  |
# |PCB    | Header       |Pins  |
# +-------+--------------+------+
# | ENA   | ??? 	        | ??   |
# | IN1   | GPIO 6	      | 31   |
# | IN2   | GPIO 13      | 33   |
# | IN3   | GPIO 19      | 35   |
# | IN4	  | GPIO 26      | 37   |
# | ENB   | ??? 	        | ??   |
# +-------+--------------+------+
# |  GND  | GND          | 39   |
# | +12V  | ExternalPower| n/a  |
# |  +5V  | NotConnected | n/a  |
# +-------+--------------+------+

"""
GPIO_R_Reverse = 5   #RPI2-pin31: "Right Reverse"
GPIO_R_Forward = 6   #RPI2-pin29: "Right Forward"
GPIO_L_Reverse = 13  #RPI2-pin37: "Left Reverse"
GPIO_L_Forward = 26  #RPI2-pin33: "Left Forward"
"""

GPIO_R_Reverse = 6   #RPI2-pin31: "Right Reverse"
GPIO_R_Forward = 13   #RPI2-pin29: "Right Forward"
GPIO_L_Reverse = 19  #RPI2-pin37: "Left Reverse"
GPIO_L_Forward = 26  #RPI2-pin33: "Left Forward"

# Set GPIO-pins as OUTput
GPIO.setup(GPIO_R_Forward,GPIO.OUT) 
GPIO.setup(GPIO_R_Reverse,GPIO.OUT) 
GPIO.setup(GPIO_L_Forward,GPIO.OUT) 
GPIO.setup(GPIO_L_Reverse,GPIO.OUT) 

def TurnLeft(angle):
    duration = angle / 125.0  # Exprimental constant
    print ('TurnLeft(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_R_Forward,True)
    time.sleep(duration)
    GPIO.output(GPIO_R_Forward,False)

def TurnRight(angle):
    duration = angle / 125.0 # Exprimental constant
    print ('TurnRight(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_L_Forward,True)
    time.sleep(duration)
    GPIO.output(GPIO_L_Forward,False)

def Forward(distance):
    duration = 0.5 # Exprimental constant
    print ('Forward(',distance,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_L_Forward,True)
    GPIO.output(GPIO_R_Forward,True)
    time.sleep(0.5)
    GPIO.output(GPIO_L_Forward,False)
    GPIO.output(GPIO_R_Forward,False)

def Reverse(distance):
    duration = 0.5 # Exprimental constant
    print ('Reverse(',distance,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_L_Reverse,True)
    GPIO.output(GPIO_R_Reverse,True)
    time.sleep(duration)
    GPIO.output(GPIO_L_Reverse,False)
    GPIO.output(GPIO_R_Reverse,False)

def ReverseTurnLeft(angle):
    duration = angle / 125.0 # Exprimental constant
    print ('ReverseTurnLeft(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_R_Reverse,True)
    time.sleep(duration)
    GPIO.output(GPIO_R_Reverse,False)

def ReverseTurnRight(angle):
    duration = angle / 125.0 # Exprimental constant
    print ('ReverseTurnRight(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_L_Reverse,True)
    time.sleep(duration)
    GPIO.output(GPIO_L_Reverse,False)

def SpinnLeft(angle):
    duration = angle / 50 # Exprimental constant
    print ('SpinnLeft(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_L_Reverse,True)
    GPIO.output(GPIO_R_Forward,True)
    time.sleep(duration)
    GPIO.output(GPIO_L_Reverse,False)
    GPIO.output(GPIO_R_Forward,False)

def SpinnRight(angle):
    duration = angle / 50 # Exprimental constant
    print ('SpinnRight(',angle,'deg) = (',duration,'sec)')
    GPIO.output(GPIO_R_Reverse,True)
    GPIO.output(GPIO_L_Forward,True)
    time.sleep(duration)
    GPIO.output(GPIO_R_Reverse,False)
    GPIO.output(GPIO_L_Forward,False)

# +-----------------------------+
# | Wrap main content in a try block so we can
# | catch the user pressing CTRL-C and run the
# | GPIO cleanup function. This will also prevent
# | the user seeing lots of unnecessary error
# | messages.
# +-----------------------------+
try:
    print ("Try...")
    SpinnRight(90)
    SpinnLeft(90)
    #Forward(10)
    #TurnLeft(90)
    #Forward(10)
    #Reverse(10)
    #ReverseTurnLeft(90)
    #TurnRight(90)
    #Forward(50)
    #Reverse(50)
    #ReverseTurnRight(90)

    print ("...End!")
    GPIO.cleanup()
    
except KeyboardInterrupt:
    # User pressed CTRL-C
    # Reset GPIO settings
    print ("Cancel! Ctrl-C pressed...")
    GPIO.cleanup()
    
except:
    print ("Error! Exiting...")
    GPIO.cleanup()
