#!/usr/bin/env python
# Behaviour:
# 1) Run straight a head.
# 2a) Rule "a": Avoid colliding with obstacle by using distance sensors to desice if turn left vs. right (e.g. Ultrasonic HC-04).
# 2b) Rule "b": Stop when detecting line by using line follower sensors.
# 2c) Display reaction on LCD
from __future__ import division

import rospy

from std_msgs.msg       import String                # As output to LCD
from pet_mk_iv_msgs.msg import DistanceMeasurement   # As input from dist.sensors 
from pet_mk_iv_msgs.msg import TripleBoolean         # As input from line follower sensors 
from geometry_msgs.msg  import TwistStamped          # AS output to velocity controller

class MissionNode(object):
    kLinearSpeed   = 0.2 # m/sec
    kRotationSpeed = 4.0 # rad/sec

    kForwardDistance = 300 # mm
    kSideDistance    = 150 # mm
 
    def __init__(self):
        rospy.init_node("mission_impossible")

        self.emergency_stop = False

        # Subscribe for Ultra Sonic distance sensors
        self.dist_sensors_left   = -1 # Infinity-"mm"
        self.dist_sensors_middle = -1 # Infinity-"mm" 
        self.dist_sensors_right  = -1 # Infinity-"mm"
        self.dist_sub = rospy.Subscriber("dist_sensors", DistanceMeasurement, self.callback_dist_senors)

        # Subscribe for Line Follower sensors
        self.LF_sensors_msg = None
        self.LF_sub = rospy.Subscriber("line_followers", TripleBoolean, self.callback_LF_senors)

        # Publishers
        self.vel_pub  = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)
        self.row1_pub = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        self.row2_pub = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        
        self.vel_msg = TwistStamped()
        self.vel_msg.header.frame_id = "base_link"  # Vehicle local coordinate frame
        
        rospy.wait_for_message("dist_sensors", DistanceMeasurement, timeout=10)
        rospy.wait_for_message("line_followers", TripleBoolean, timeout=10)

        return

    def start(self):
        self.row1_pub.publish("STARTED")
        self.row2_pub.publish("...tjoho!..")

        self.check_for_stop_timer  = rospy.Timer(rospy.Duration(0, 100000000), self.check_for_stop)
        self.avoid_obstacles_timer = rospy.Timer(rospy.Duration(0, 100000000), self.avoid_obstacles)

        return

    def check_for_stop(self, msg):
        # Stop criteria #1 ...
        if not (self.LF_sensors_msg.left and self.LF_sensors_msg.middle and self.LF_sensors_msg.right):
            rospy.logwarn("STOP! <= Line detected")
            self.row1_pub.publish("STOP")
            self.emergency_stop = True

        # Stop criteria #2 ...       
        if (0 < self.dist_sensors_middle < MissionNode.kForwardDistance and 
            0 < self.dist_sensors_left   < MissionNode.kSideDistance and
            0 < self.dist_sensors_right  < MissionNode.kSideDistance ):
            rospy.logwarn("STOP! <= Obstacles all over the place")
            self.row1_pub.publish("STOP")
            self.row2_pub.publish("..road blocked..")
            self.emergency_stop = True
        
        return

    def avoid_obstacles(self, msg):
        if (self.emergency_stop):
            self.vel_msg.twist.linear.x  = 0.0 # STOP! No propulsion
            self.vel_msg.twist.angular.z = 0.0 # STOP! No twist/turn

            rospy.signal_shutdown("Mission accomplished")

        elif (0 < self.dist_sensors_middle < MissionNode.kForwardDistance):
            rospy.loginfo("Obstacle detected - Straight ahead")
            self.row2_pub.publish("Obstacle-Front")
            #Obstacle straight ahead => Turn Left or Right?
            self.vel_msg.twist.linear.x = 0.0
     
            if (self.dist_sensors_left > self.dist_sensors_right):
                self.vel_msg.twist.angular.z = self.kRotationSpeed  #Turn Left CCW
            else: 
                self.vel_msg.twist.angular.z = -self.kRotationSpeed #Turn Right CW
                
        elif (0 < self.dist_sensors_left < MissionNode.kSideDistance):
            rospy.loginfo("Obstacle detected - To the Left")
            self.row2_pub.publish("Obstacle-LEFT")
            self.vel_msg.twist.linear.x = 0.0
            self.vel_msg.twist.angular.z = -self.kRotationSpeed #Turn Right CW

            
        elif (0 < self.dist_sensors_right < MissionNode.kSideDistance ):
            rospy.loginfo("Obstacle detected - To the Right")
            self.row2_pub.publish("Obstacle-RIGHT")
            self.vel_msg.twist.linear.x = 0.0
            self.vel_msg.twist.angular.z = self.kRotationSpeed # Turn Left CCW

        else:
            rospy.loginfo("Run Forrest... RUN!")
            self.row2_pub.publish("Run Forrest,RUN!")
            self.vel_msg.twist.linear.x = self.kLinearSpeed 
            self.vel_msg.twist.angular.z = 0.0 

        self.vel_msg.header.stamp = rospy.Time.now()  # Need to set timestamp in message header before publish.
        self.vel_pub.publish(self.vel_msg)
        return       

    def callback_dist_senors(self, msg): 
        if   msg.header.frame_id == "dist_sensor_right": 
           self.dist_sensors_right = msg.distance        # mm
           
        elif msg.header.frame_id == "dist_sensor_mid":
           self.dist_sensors_middle = msg.distance       # mm
           
        elif msg.header.frame_id == "dist_sensor_left":
           self.dist_sensors_left = msg.distance         # mm

    def callback_LF_senors(self, msg): 
        self.LF_sensors_msg = msg

if __name__ == '__main__':
    mission_impossible = MissionNode()
    mission_impossible.start()
    rospy.spin()