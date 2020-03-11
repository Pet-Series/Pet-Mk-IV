#!/usr/bin/env python
# Github @SeniorKullken 2020-03-11
#
# Linux/Raspian status/information on LCD-Display
# Route msg that is published on ROS-topic "lcd_display/row1" => subscriber LCD-display Row1
# Route msg that is published on ROS-topic "lcd_display/row2" => subscriber LCD-display Row2
#
# $ rosrun ros_lcd_driver lcd_driver_node.py
#
# -----Prerequisite------------------------------
# Hardware: Display: 1602 16x2 Character LCD Backlight
# Hardware: I2C interface: PC8574T
# Operating system: Linux/Raspian Buster (based on Debian 10)
# Middleware: ROS 1.0 Melodic
# Python 2
#
# $ sudo i2cdetect -y 1
#     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
#  00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 3f 
#  40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#  70: -- -- -- -- -- -- -- -- 
#
from __future__ import division
import rospy
from std_msgs.msg import String
from rpi_lcd import LCD

class LCDDisplay(object): # A ROS-subscriber node...

    def __init__(self):
        rospy.init_node("lcd_display")
        self.update_rate = rospy.Rate(1) #1Hz
        
        #Initiate the SSD1306 LCD-display with a PC8574T piggy back I2C-interface
        self.lcd = LCD(address=0x3f, width=16, rows=2)
        
        # Default/Init.values
        self.row1_string = "Init.LCD row1" # Initial text on LCD Display
        self.row2_string = "Init.LCD row2" #     ---- " -----
        
        # Subscribers
        self.row1_sub = rospy.Subscriber("~row1", String, self.row1_cb)
        self.row2_sub = rospy.Subscriber("~row2", String, self.row2_cb)
  
        # Publishers
        # n/a

    def run(self):
        #
        while not rospy.is_shutdown():
            #
            # stuff... Uppdatera LCD-display
            self.lcd.text(self.row1_string, 1, 'center')
            self.lcd.text(self.row2_string, 2, 'center')
            # Debug text to terminal
            rospy.loginfo(rospy.get_caller_id() + "LCD-driver Row1=" + self.row1_string ) 
            rospy.loginfo(rospy.get_caller_id() + "LCD-driver Row2=" + self.row2_string )
            # Put myself into sleep...Zzzz
            self.update_rate.sleep()
            
    def row1_cb(self,msg):
        # Subscriber Callback - Someone have published a message on topic "lcd_display/row1"
        self.row1_string = msg.data
     
    def row2_cb(self,msg):
        # Subscriber Callback - Someone have published a message on topic "lcd_display/row2"
        self.row2_string = msg.data

if __name__ == '__main__':
    lcd_display_node = LCDDisplay()
    lcd_display_node.run()