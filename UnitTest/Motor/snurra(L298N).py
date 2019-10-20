#!/usr/bin/python
# Sample Python3 App for L298N - Dual H Bridge DC Stepper Motor Drive Controller Board Module
# Sites I used...
#  http://explainingcomputers.com/rasp_pi_robotics.html
#  http://www.raspberry-pi-car.com/top-story-en/raspberry-pi-wifi-radio-controlled-rc-vehicle-wiring/6928?lang=en&lang=en
#  http://www.raspberry-pi-car.com/top-story-en/raspberry-pi-motor-control-l298n-h-bridge-programming/7342?lang=en
#
# +-----------------------------+
# |Import required Python libraries
# +-----------------------------+
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
# | ENA   | ??? 	 | ??   |
# | IN1   | GPIO5	 | 29   |
# | IN2   | GPIO6        | 31   |
# | IN3   | GPIO13       | 33   |
# | IN4	  | GPIO26       | 37   |
# | ENB   | ??? 	 | ??   |
# +-------+--------------+------+
# |  GND  | GND          | 39   |
# | +12V  | ExternalPower| n/a  |
# |  +5V  | NotConnected | n/a  |
# +-------+--------------+------+
GPIO_R_Forward = 5   #RPI2-pin29: "Right Forward"
GPIO_R_Reverse = 6   #RPI2-pin31: "Right Reverse"
GPIO_L_Forward = 13  #RPI2-pin33: "Left Forward"
GPIO_L_Reverse = 26  #RPI2-pin37: "Left Reverse"
# Set GPIO-pins as OUTput
GPIO.setup(GPIO_R_Forward,GPIO.OUT) 
GPIO.setup(GPIO_R_Reverse,GPIO.OUT) 
GPIO.setup(GPIO_L_Forward,GPIO.OUT) 
GPIO.setup(GPIO_L_Reverse,GPIO.OUT) 

# +-----------------------------+
# | Wrap main content in a try block so we can
# | catch the user pressing CTRL-C and run the
# | GPIO cleanup function. This will also prevent
# | the user seeing lots of unnecessary error
# | messages.
# +-----------------------------+
try:
    print ("Try...")
    print ("GPIO.setup(5,GPIO.OUT) #Pin29: RF Forward")
    GPIO.output(GPIO_R_Forward,True)
    time.sleep(0.5)
    GPIO.output(GPIO_R_Forward,False)

    print ("GPIO.setup(6,GPIO.OUT) #Pin31: RF Reverse")
    GPIO.output(GPIO_R_Reverse,True)
    time.sleep(0.5)
    GPIO.output(GPIO_R_Reverse,False)

    print ("GPIO.setup(13,GPIO.OUT)#Pin33: LF Forward")
    GPIO.output(GPIO_L_Forward,True)
    time.sleep(0.5)
    GPIO.output(GPIO_L_Forward,False)

    print ("GPIO.setup(26,GPIO.OUT)#Pin37: LF Reverse")
    GPIO.output(GPIO_L_Reverse,True)
    time.sleep(0.5)
    GPIO.output(GPIO_L_Reverse,False)

    print ("END")
    GPIO.cleanup()
    
except KeyboardInterrupt:
    # User pressed CTRL-C
    # Reset GPIO settings
    print ("Cancel! Ctrl-C pressed...")
    GPIO.cleanup()
