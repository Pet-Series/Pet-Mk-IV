#!/usr/bin/env python
# Pet Mk. IV
# Behaviour
# 1) Run straight a head.
# 2a) Rule "a": Avoid colliding with obstacle by using distance sensors to desice if turn left vs. right (e.g. Ultrasonic HC-04).
# 2b) Rule "b": Stop when detecting line by using line follower sensors.
# 

from __future__ import division

import rospy


from pet_mk_iv_msgs.msg import DistanceMeasurement   # dist_sensor_right ..._mid ..._right
from pet_mk_iv_msgs.msg import TripleBoolean         # LF_sensors_msg.left /.middle / .right
from geometry_msgs.msg  import TwistStamped          # Linear velocity + Angular velocity + Header [aka. "Stamped" using rospy.Time.now()].

class ObstacleDetectStopTurn(object):
    linearSpeed = 0.2
    rotationSpeed = 4.0
    def __init__(self):
        rospy.init_node("obstacle_detect_line_stop")
        self.cmd_rate = rospy.Rate(10) #10Hz

        # Subscribe for Ultra Sonic distance sensors
        # - Default/Init.values
        self.dist_sensors_left   = -1 # Infinity-"mm"
        self.dist_sensors_middle = -1 # Infinity-"mm" 
        self.dist_sensors_right  = -1 # Infinity-"mm"

        self.dist_sub = rospy.Subscriber("dist_sensors", DistanceMeasurement, self.dist_senors_cb)
               
        # - Subscribers constraint / dependency
        #   - Does not make sense to go beyond this point before first sensor data is published.
        #     (@raise ROSException: if specified timeout is exceeded) 
        rospy.wait_for_message("dist_sensors", DistanceMeasurement, timeout=10)

        # Subscribe for Line Follower sensors
        # - Default/Init.values
        self.LF_sensors_msg = None

        self.LF_sub = rospy.Subscriber("line_followers", TripleBoolean, self.LF_senors_cb)
        # - Subscribers constraint / dependency
        #   - Does not make sense to go beyond this point before first sensor data is published.
        rospy.wait_for_message("line_followers", TripleBoolean, timeout=10)

        # Publish topic to Engine Controller(pwm) via "controller"..
        self.vel_pub = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)
        
        self.vel_msg = TwistStamped()
        self.vel_msg.twist.linear.x = 0 # Forward/Reverse Speed rel.Vehicle local cord. system.
        self.vel_msg.twist.linear.y = 0 # Sideways = "almost" always 0 rad/sec (Not in use for ground based vehicles with "normal wheels)
        self.vel_msg.twist.linear.z = 0 # Jump     = Always 0 m/sec (Not in use for ground based vehicles)
        
        self.vel_msg.twist.angular.x = 0 # Role  = Always 0 rad/sec (Not in use for ground based vehicles)
        self.vel_msg.twist.angular.y = 0 # Pitch = Always 0 rad/sec (Not in use for ground based vehicles)
        self.vel_msg.twist.angular.z = 0 # Yaw   = Turn speed (negative/- > ccw "to the left" | positive/+ > cw
        
        self.emergency_stop = False

    def run(self):
 
        while not rospy.is_shutdown() and not self.emergency_stop:
                
            # stuff

            if (0 < self.dist_sensors_middle < 300 and 
                0 < self.dist_sensors_left   < 150 and
                0 < self.dist_sensors_right  < 150 ):
                rospy.logwarn("STOP! <= Obstacle all over the place")
                self.vel_msg.twist.linear.x  = 0.0 # STOP! No propulsion
                self.vel_msg.twist.angular.z = 0.0 # STOP! No twist/turn
                self.emergency_stop = True

                
            if (0 < self.dist_sensors_middle < 300):
                #Obstacle straight ahead => Turn Left or Right
                self.vel_msg.twist.linear.x = 0.0   #STOP!
                rospy.loginfo("Obstacle detected - Straight ahead")
                    
                if (self.dist_sensors_left > self.dist_sensors_right):
                    self.vel_msg.twist.angular.z = self.rotationSpeed #Turn Left CCW
                if (self.dist_sensors_left <= self.dist_sensors_right):
                    self.vel_msg.twist.angular.z = -self.rotationSpeed #Turn Right CW
                    
            elif (0 < self.dist_sensors_left   < 150):
               #Obstacle to the Left => Turn Right CCW
                self.vel_msg.twist.linear.x = 0.0   #STOP!
                self.vel_msg.twist.angular.z = -self.rotationSpeed #Turn Right CW (rad/s)
                rospy.loginfo("Obstacle detected - To the Left")
                
            elif (0 < self.dist_sensors_right  < 150 ):
                #Obstacle to the Right => Turn Left CCW
                self.vel_msg.twist.linear.x = 0.0   #STOP!
                rospy.loginfo("Obstacle detected - To the Right")
                self.vel_msg.twist.angular.z = self.rotationSpeed # Turn Left CCW (rad/s)

            else:
                  rospy.loginfo("Run Forrest... RUN!")
                  self.vel_msg.twist.linear.x = self.linearSpeed  # m/s
                  self.vel_msg.twist.angular.z = 0.0 # rad/s

            # Last check before going further... Is a line detected?
            # ...if so - This paragraph will overide the velocity set above.
            if self.LF_sensors_msg.left and self.LF_sensors_msg.middle and self.LF_sensors_msg.right:
                rospy.logwarn_throttle(1, " ..no line detected: Run Forrest... RUN!")
            else:
                rospy.logwarn("STOP! <= Line detected")
                self.vel_msg.twist.linear.x  = 0.0 # STOP! No propulsion
                self.vel_msg.twist.angular.z = 0.0 # STOP! No twist/turn
                self.emergency_stop = True

            self.vel_msg.header.stamp = rospy.Time.now()  # Need to set timestamp in message header before publish.
            self.vel_pub.publish(self.vel_msg)
            self.cmd_rate.sleep()

    def dist_senors_cb(self, msg):  # "*_cb" = "*_call back"
   
        if   msg.header.frame_id == "dist_sensor_right": 
           self.dist_sensors_right = msg.distance        # mm
           
        elif msg.header.frame_id == "dist_sensor_mid":
           self.dist_sensors_middle = msg.distance       # mm
           
        elif msg.header.frame_id == "dist_sensor_left":
           self.dist_sensors_left = msg.distance         # mm

    def LF_senors_cb(self, msg):  # "*_cb" = "*_call back"
        self.LF_sensors_msg = msg

if __name__ == '__main__':
    Obstacle_Detect_Stop_Turn = ObstacleDetectStopTurn()
    Obstacle_Detect_Stop_Turn.run()
