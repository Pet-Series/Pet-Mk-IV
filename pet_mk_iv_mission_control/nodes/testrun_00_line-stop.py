#!/usr/bin/env python
# Pet-Mk-IV ROS1 mission
#  \licens       Software License Agreement (MIT)
#  \authors      stefan.kull@gmail.com (Github ID 'SeniorKullken')
#  \repository   https://github.com/Pet-Series
#  \repository   https://github.com/Pet-Series/Pet-Mk-IV
#  \description  Complete vehicle test level. Line senors & motor will be active.
#
# Behaviour
# 1. Run in a straight line.
# 2. When line detected - Then just stop!
#
# Launch sequence:
# 1. $ roslaunch pet_mk_iv_simulation pet_play_yard-02.launch
# 2. $ rosrun pet_mk_iv_mission_control testrun_00_line-stop.py 
#
# TODO: Parametric linesAre=LineDetection.DARK or linesAre=LineDetection.LIGHT
#
from __future__ import division

import rospy
from pet_mk_iv_msgs.msg import LineDetection
from geometry_msgs.msg  import TwistStamped  # Linear velocity + Angular velocity
#from geometry_msgs.msg  import Twist         # Linear velocity + Angular velocity

class LineFollower(object):

    def __init__(self):
        rospy.init_node("line_detection_test")
        self.cmd_rate = rospy.Rate(100) # Hz

        # Subscribers
        rospy.logwarn_throttle(1, " Setting up Subscribers...")  

        self.line_sensor_right  = None
        self.line_sensor_middle = None
        self.line_sensor_left   = None

        self.line_sensor_right_sub  = rospy.Subscriber("/line_sensor/right",  LineDetection, self.callback_line_sensor_right )
        self.line_sensor_middle_sub = rospy.Subscriber("/line_sensor/middle", LineDetection, self.callback_line_sensor_middle)
        self.line_sensor_left_sub   = rospy.Subscriber("/line_sensor/left",   LineDetection, self.callback_line_sensor_left  )
        rospy.wait_for_message("/line_sensor/middle", LineDetection, timeout=10)

        # Publishers
        rospy.logwarn_throttle(1, " Setting up Publishers...") 
        self.vel_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10) #  TwistStamped vs. Twist

    def run(self):
        vel_msg = TwistStamped()  #  TwistStamped vs. Twist

        vel_msg.twist.linear.x = 0 # Speed forward
        vel_msg.twist.linear.y = 0 # "almost" always 0 rad/sec
        vel_msg.twist.linear.z = 0 # Always 0 m/sec
        
        vel_msg.twist.angular.x = 0 # Always 0 rad/sec
        vel_msg.twist.angular.y = 0 # Always 0 rad/sec
        vel_msg.twist.angular.z = 0 # Turn speed (ccw "to the left")
     
        emergency_stop = False

        rospy.logwarn_throttle(1, " Run sooon... before while:")  

        while not rospy.is_shutdown() and not emergency_stop:

            vel_msg.header.stamp = rospy.Time.now()
            
            # stuff
            if self.line_sensor_right and self.line_sensor_middle_msg and self.line_sensor_left:
                rospy.logwarn_throttle(1, " Run Forrest... RUN!")
                vel_msg.twist.linear.x = 0.5
            else:
                rospy.logwarn(" STOP!")
                vel_msg.twist.linear.x = 0.0
                emergency_stop = True
                
            self.vel_pub.publish(vel_msg)        
            self.cmd_rate.sleep()

    def callback_line_sensor_right(self, msg):
        self.line_sensor_right = msg.value

    def callback_line_sensor_middle(self, msg):
        self.line_sensor_middle_msg = msg.value

    def callback_line_sensor_left(self, msg):
        self.line_sensor_left = msg.value

if __name__ == '__main__':
    line_follower = LineFollower()
    line_follower.run()
