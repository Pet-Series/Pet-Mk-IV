#!/usr/bin/env python
# $ rosrun ros_lcd_driver lcd_driver_node.py
# ...turn.py
# Publishers
        #self.row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        #self.row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        #
        # self.row1_pub.publish(String("Hello"))
        # self.row2_pub.publish(String("World"))
#   - 
from __future__ import division

import rospy

from std_msgs.msg import String

class LCDDisplay(object):

    def __init__(self):
        rospy.init_node("lcd_display")
        self.update_rate = rospy.Rate(1) #1Hz

        # Subscribers
        self.row1_sub = rospy.Subscriber("~row1", String, self.row1_cb)
        self.row2_sub = rospy.Subscriber("~row2", String, self.row2_cb)
  
        # Publishers
        #self.vel_pub = rospy.Publisher("vel_cmd", Twist, queue_size=10)

    def run(self):
        #
        while not rospy.is_shutdown()
            #
            # stuff
            #
            self.update_rate.sleep()
            
    def row1_cb(self,msg):
        #
        # self.row1_string = msg.data
        
    def row2_cb(self,msg):
        #
        # self.row1_string = msg.data

if __name__ == '__main__':
    lcd_display_node = LCDDisplay()
    lcd_display_node.run()

