#!/usr/bin/env python
# Github @SeniorKullken 2020-03-11
#
# On ROS-topic "lcd_display/row1" - Publish current date-time on    ...example "11 Mar 19:20:36"
# on ROS-topic "lcd_display/row2" - Publish how seconds since start ...example  "Run 56.00 Sec"
#
# $ rosrun ros_lcd_driver lcd_driver_node.py
#
# -----Prerequisite------------------------------
# Operating system: Linux/Raspian Buster (based on Debian 10)
# Middleware: ROS 1.1x Melodic
# 
# $ rostopic list
#      /lcd_display/row1
#      /lcd_display/row2

import rospy
import datetime
from std_msgs.msg import String

class LCDPublisher(object): # A ROS-publisher 

    def __init__(self):
        rospy.init_node("publish2LCD")
        self.publish_rate = rospy.Rate(1) # 10hz
        self.start_time  = rospy.get_time()
        #
        # Publishers
        self.row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        self.row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        #
        # Subscribers
        # n/a  

    def run(self):
        while not rospy.is_shutdown():
            # Create content to be published
            self.time_now  = datetime.datetime.now().strftime("%d %b %H:%M:%S") # Why  not using rospy.get_time() ?
            self.time_past = rospy.get_time() - self.start_time
            # Format the content as strings before published
            self.row1_str = "%s"   % self.time_now          # Syntax "11 Mar 19:20:36"
            self.row2_str = "Run %.1f sec" % self.time_past # Syntax "Run 56.00 Sec"
            # Debug text to terminal            
            rospy.loginfo("lcd_display/row1=\"" + self.row1_str + "\"")
            rospy.loginfo("lcd_display/row2=\"" + self.row2_str + "\"")
            # Publish!
            self.row1_pub.publish(self.row1_str)
            self.row2_pub.publish(self.row2_str)
            # Put myself into sleep...Zzzz
            self.publish_rate.sleep()

if __name__ == '__main__':
    lcd_publish_node = LCDPublisher()
    lcd_publish_node.run()