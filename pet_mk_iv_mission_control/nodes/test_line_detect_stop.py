#!/usr/bin/env python

from __future__ import division

import rospy
# <Turn Left> "Line to the Left"
# <Forward> "On track"
# <Turn Right> "Line to the Right"
# <Stop> "Finish"
# <Stop> "Undefined"

# bool left
# bool middle
# bool right
from pet_mk_iv_msgs.msg import TripleBoolean
from geometry_msgs.msg import TwistStamped  # Linear velocity + Angular velocity

class LineFollower(object):

    def __init__(self):
        rospy.init_node("line_follower_state_machine")
        self.cmd_rate = rospy.Rate(100) # Hz

        # Subscribers
        self.LF_sensors_msg = None
        self.LF_sub = rospy.Subscriber("line_followers", TripleBoolean, self.LF_senors_cb)
        rospy.wait_for_message("line_followers", TripleBoolean, timeout=10)

        # Publishers
        self.vel_pub = rospy.Publisher("vel_cmd", TwistStamped, queue_size=10)

    def run(self):
        vel_msg = TwistStamped()

        vel_msg.header.stamp = rospy.Time.now()

        vel_msg.twist.linear.x = 0 # Speed forward
        vel_msg.twist.linear.y = 0 # "almost" always 0 rad/sec
        vel_msg.twist.linear.z = 0 # Always 0 m/sec
        
        vel_msg.twist.angular.x = 0 # Always 0 rad/sec
        vel_msg.twist.angular.y = 0 # Always 0 rad/sec
        vel_msg.twist.angular.z = 0 # Turn speed (ccw "to the left")
        
        emergency_stop = False
        
        while not rospy.is_shutdown() and not emergency_stop:
            
            # stuff
            if self.LF_sensors_msg.left and self.LF_sensors_msg.middle and self.LF_sensors_msg.right:
                rospy.logwarn_throttle(1, rospy.get_caller_id() + " Run Forrest... RUN!")
                vel_msg.twist.linear.x = 0.1
            else:
                rospy.logwarn(rospy.get_caller_id() + " STOP!")
                vel_msg.twist.linear.x = 0.0
                emergency_stop = True
                
            self.vel_pub.publish(vel_msg)        
            self.cmd_rate.sleep()

    def LF_senors_cb(self, msg):
        self.LF_sensors_msg = msg

if __name__ == '__main__':
    line_follower = LineFollower()
    line_follower.run()
