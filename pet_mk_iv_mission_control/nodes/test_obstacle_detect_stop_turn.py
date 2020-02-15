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

class ObstacleDetectStopTurn(object):

    def __init__(self):
        rospy.init_node("obstacle_detect_stop_turn")
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
        vel_msg.linear.x = 0 # Forward/Reverse Speed rel.Vehicle local cord. system.
        vel_msg.linear.y = 0 # Sideways = "almost" always 0 rad/sec (Not in use for ground based vehicles with "normal wheels)
        vel_msg.linear.z = 0 # Djump    = Always 0 m/sec (Not in use for ground based vehicles)
        
        vel_msg.angular.x = 0 # Role  = Always 0 rad/sec (Not in use for ground based vehicles)
        vel_msg.angular.y = 0 # Pitch = Always 0 rad/sec (Not in use for ground based vehicles)
        vel_msg.angular.z = 0 # Yaw   = Turn speed (negative/- > ccw "to the left" | positive/+ > cw
        
        emergency_stop = False
        
        while not rospy.is_shutdown() and not emergency_stop:
                
            # stuff
            if (0 < self.dist_sensors_middle < 300 and 
                0 < self.dist_sensors_left   < 150 and
                0 < self.dist_sensors_right  < 150 ):
                #Obstacle all over the place => STOP!
                vel_msg.linear.x = 0.0   #STOP!
                rospy.logwarn(rospy.get_caller_id() + "Obstacle all over the place => STOP!")
                emergency_stop = True

                
            if (0 < self.dist_sensors_middle < 300):
                #Obstacle straight ahead => Turn Left or Right
                vel_msg.linear.x = 0.0   #STOP!
                rospy.loginfo(rospy.get_caller_id() + "Obstacle detected - Straight ahead")
                    
                if (self.dist_sensors_left > self.dist_sensors_right):
                    vel_msg.angular.z = 1.5 #Turn Left CCW
                if (self.dist_sensors_left <= self.dist_sensors_right):
                    vel_msg.angular.z = -1.5 #Turn Right CW
                    
            elif (0 < self.dist_sensors_left   < 150):
               #Obstacle to the Left => Turn Right CCW
                vel_msg.linear.x = 0.0   #STOP!
                vel_msg.angular.z = -1.5 #Turn Right CW
                rospy.loginfo(rospy.get_caller_id() + "Obstacle detected - To the Left")
                
            elif (0 < self.dist_sensors_right  < 150 ):
                #Obstacle to the Right => Turn Left CCW
                vel_msg.linear.x = 0.0   #STOP!
                rospy.loginfo(rospy.get_caller_id() + "Obstacle detected - To the Right")
                vel_msg.angular.z = 1.5 #Turn Left CCW
                #emergency_stop = True
            else:
                  rospy.loginfo(rospy.get_caller_id() + " Run Forrest... RUN!")
                  vel_msg.linear.x = 0.1
                  vel_msg.angular.z = 0.0
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
    Obstacle_Detect_Stop_Turn = ObstacleDetectStopTurn()
    Obstacle_Detect_Stop_Turn.run()
