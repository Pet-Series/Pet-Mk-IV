#!/usr/bin/env python

import rospy
import datetime
from std_msgs.msg import String

def publish2LCD():
#     self.row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
#     self.row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
    row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
    row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
    rospy.init_node("publish2LCD")
    start_time  = rospy.get_time()

#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        time_now  = datetime.datetime.now().strftime("%d %b %H:%M:%S") # rospy.get_time()
        time_past = rospy.get_time() - start_time
        
        row1_str = "Row1 %s" % time_now
        row2_str = "Row2 %.1f" % time_past
        rospy.loginfo(row1_str)
        rospy.loginfo(row2_str)
#         self.row1_pub.publish(row1_str)
#         self.row2_pub.publish(row2_str)
        row1_pub.publish(row1_str)
        row2_pub.publish(row2_str)
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish2LCD()
    except rospy.ROSInterruptException:
        pass