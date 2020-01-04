#!/usr/bin/env python

from __future__ import division
import atexit

import rospy
# <Turn Left> "Line to the Left"
# <Forward> "On track"
# <Turn Right> "Line to the Right"
# <Stop> "Finish"
# <Stop> "Undefined"

from pet_mk_iv_msgs.msg import DistanceMeasurement
from geometry_msgs.msg import Twist  # Linear velocity + Angular velocity

class TestObstacleDetectStop(object):

    def __init__(self):
        rospy.init_node("test_obstacle_detect_stop")
        self.cmd_rate = rospy.Rate(10) #10Hz

        # Subscribers
        self.dist_sensors_left   = -1 # Infinity-"mm"
        self.dist_sensors_middle = -1 # Infinity-"mm" 
        self.dist_sensors_right  = -1 # Infinity-"mm" 

        self.dist_sub = rospy.Subscriber("dist_sensors", DistanceMeasurement, self.dist_senors_cb)
        # rospy.wait_for_message("dist_sensors", DistanceMeasurement, timeout=10)

        # Publishers
        self.vel_pub = rospy.Publisher("vel_cmd", Twist, queue_size=10)

    def run(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0 # Speed forward
        vel_msg.linear.y = 0 # "almost" always 0 rad/sec
        vel_msg.linear.z = 0 # Always 0 m/sec
        
        vel_msg.angular.x = 0 # Always 0 rad/sec
        vel_msg.angular.y = 0 # Always 0 rad/sec
        vel_msg.angular.z = 0 # Turn speed (ccw "to the left")
        
        emergency_stop = False
        
        while not rospy.is_shutdown() and not emergency_stop:
            
            # stuff
            if (0 < self.dist_sensors_middle < 300 or 
                0 < self.dist_sensors_left   < 150 or
                0 < self.dist_sensors_right  < 150 ):
                  rospy.logwarn(rospy.get_caller_id() + " STOP! Obstacle detected")
                  vel_msg.linear.x = 0.0
                  emergency_stop = True
            else:
                  rospy.logwarn(rospy.get_caller_id() + " Run Forrest... RUN!")
                  vel_msg.linear.x = 0.1
                  emergency_stop = False

            self.vel_pub.publish(vel_msg)
            self.cmd_rate.sleep()

    def dist_senors_cb(self, msg):  # "*_cb" = "*_call back"
   
        if   msg.header.frame_id == "dist_sensor_0":
           self.dist_sensors_right = msg.distance
           
        elif msg.header.frame_id == "dist_sensor_1":
           self.dist_sensors_middle = msg.distance
           
        elif msg.header.frame_id == "dist_sensor_2":
           self.dist_sensors_left = msg.distance
        
        rospy.loginfo(rospy.get_caller_id() + "I heard; Left={}mm, Middle={}mm, Right={}mm".format(
                        self.dist_sensors_left,self.dist_sensors_middle,self.dist_sensors_right)
                      )

if __name__ == '__main__':
    Obstacle_Detect_Stop = TestObstacleDetectStop()
    Obstacle_Detect_Stop.run()
