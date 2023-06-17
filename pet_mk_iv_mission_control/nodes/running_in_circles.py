#!/usr/bin/env python
# Status: ...not tested on latest platform.

import rospy

from geometry_msgs.msg import TwistStamped

class CircleRunner(object):

    def __init__(self):
        rospy.init_node("circle_runner")
        self.cmd_rate = rospy.Rate(50) # Hz

        # Publishers
        self.vel_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)
        self.vel_msg = TwistStamped()
        self.vel_msg.header.frame_id = "base_link"
        self.vel_msg.twist.linear.x = 0.2
        self.vel_msg.twist.angular.z = 4.5

    def run(self):
        while not rospy.is_shutdown():
            self.vel_msg.header.stamp = rospy.Time.now()
                
            self.vel_pub.publish(self.vel_msg)        
            self.cmd_rate.sleep()

if __name__ == '__main__':
    node = CircleRunner()
    node.run()
