#!/usr/bin/env python
# Pet Mk. IV
# Behaviour
# 1. Run in a straight line.
# 2. When line obstacle in front of vehicle - Then just stop!
#
from __future__ import division

import rospy

from pet_mk_iv_msgs.msg import DistanceMeasurement
from geometry_msgs.msg import TwistStamped          # Linear velocity + Angular velocity + Header [aka. "Stamped" using rospy.Time.now()].

class TestObstacleDetectStop(object):

    def __init__(self):
        rospy.init_node("test_obstacle_detect_stop")
        self.cmd_rate = rospy.Rate(10) #10Hz

        # Default/Init.values
        self.dist_sensors_left   = -1 # Infinity-"mm"
        self.dist_sensors_middle = -1 # Infinity-"mm" 
        self.dist_sensors_right  = -1 # Infinity-"mm" 

        # Subscribers
        self.dist_sub = rospy.Subscriber("dist_sensors", DistanceMeasurement, self.dist_senors_cb)
        
        # Subscribers constraint / dependency
        # - Does not make sense to go beyond this point before first sensor data is published.
        #   (@raise ROSException: if specified timeout is exceeded) 
        rospy.wait_for_message("dist_sensors", DistanceMeasurement, timeout=10)

        # Publishers
        self.vel_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)

    def run(self):
        vel_msg = TwistStamped()
        vel_msg.twist.linear.x = 0 # Forward/Reverse Speed rel.Vehicle local cord. system.
        vel_msg.twist.linear.y = 0 # Sideways = "almost" always 0 rad/sec (Not in use for ground based vehicles with "normal wheels)
        vel_msg.twist.linear.z = 0 # Jump    = Always 0 m/sec (Not in use for ground based vehicles)
        
        vel_msg.twist.angular.x = 0 # Role  = Always 0 rad/sec (Not in use for ground based vehicles)
        vel_msg.twist.angular.y = 0 # Pitch = Always 0 rad/sec (Not in use for ground based vehicles)
        vel_msg.twist.angular.z = 0 # Yaw   = Turn speed (negative/- > ccw "to the left" | positive/+ > cw
        
        emergency_stop = False
        
        while not rospy.is_shutdown() and not emergency_stop:
            
            # stuff
            if (0 < self.dist_sensors_middle < 300 or 
                0 < self.dist_sensors_left   < 150 or
                0 < self.dist_sensors_right  < 150 ):
                  rospy.logwarn(" STOP! Obstacle detected")
                  self.dist_sub.unregister() # Disable subscriber.
                  vel_msg.twist.linear.x = 0.0
                  emergency_stop = True
            else:
                  rospy.loginfo_once(" Run Forrest... RUN!")
                  vel_msg.twist.linear.x = 0.2
                  emergency_stop = False
            
            vel_msg.header.stamp = rospy.Time.now()  # Need to set timestamp in message header before publish.
            self.vel_pub.publish(vel_msg)
            self.cmd_rate.sleep()

    def dist_senors_cb(self, msg):  # "*_cb" = "*_call back"
   
        if   msg.header.frame_id == "dist_sensor_right": 
           self.dist_sensors_right = msg.distance        # mm
           
        elif msg.header.frame_id == "dist_sensor_mid":
           self.dist_sensors_middle = msg.distance       # mm
           
        elif msg.header.frame_id == "dist_sensor_left":
           self.dist_sensors_left = msg.distance         # mm
        
        rospy.loginfo_throttle(0.2, "I heard; Left={}mm, Middle={}mm, Right={}mm".format(
                        self.dist_sensors_left,self.dist_sensors_middle,self.dist_sensors_right)
                      )

if __name__ == '__main__':
    Obstacle_Detect_Stop = TestObstacleDetectStop()
    Obstacle_Detect_Stop.run()
