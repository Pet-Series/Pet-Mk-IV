#!/usr/bin/env python
# $ rosrun ros_lcd_driver lcd_driver_node.py
# ...turn.py
# Publishers (via Python)
        #self.row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        #self.row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        #
        # self.row1_pub.publish(String("Hello"))
        # self.row2_pub.publish(String("World"))
# Publishers (via cmd/sh/Terminal)
#   -

from __future__ import division

import rospy

from std_msgs.msg import String

class LCDDisplay(object):

    def __init__(self):
        rospy.init_node("lcd_display")
        self.update_rate = rospy.Rate(1) #1Hz
        
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
            # stuff
            rospy.loginfo(rospy.get_caller_id() + "LCD-driver Row1=" + self.row1_string )
            rospy.loginfo(rospy.get_caller_id() + "LCD-driver Row2=" + self.row2_string )
            #
            self.update_rate.sleep()
            
    def row1_cb(self,msg):
        #
        self.row1_string = msg.data

        
    def row2_cb(self,msg):
        #
        self.row2_string = msg.data


if __name__ == '__main__':
    lcd_display_node = LCDDisplay()
    lcd_display_node.run()