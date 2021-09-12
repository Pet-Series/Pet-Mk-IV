#!/usr/bin/env python
# Behaviour:
# 1) IR-remote [Play], [Pause] and [Stop] overroule them all
# 1) Run straight a head.
# 2a) Rule "a": Avoid colliding with obstacle by using Range sensors to decide if turn left vs. right (e.g. Ultrasonic HC-SR04).
# 2b) Rule "b": Stop when detecting any line by using line detection sensors.
# 2c) Display selected action on LCD-display.
#
# Launch sequence:
# 1. $ roslaunch pet_mk_iv_simulation pet_play_yard-03.launch
# 2. $ rosrun pet_mk_iv_mission_control testrun_01_gazebo.py 
# 3. $ rosrun topic_tools transform /cmd_vel /gazebo/cmd_vel geometry_msgs/Twist 'm.twist'
#      This command must be be started AFTER someone publish /cmd_vel (TwistStamped)
from __future__ import division

import rospy


from std_msgs.msg       import String          # As output to LCD
from pet_mk_iv_msgs.msg import LineDetection   # As input from line detector sensors
from sensor_msgs.msg    import Range           # As input from dist/range sensors       
from geometry_msgs.msg  import TwistStamped    # As output to velocity controller

from pet_mk_iv_msgs.msg import LightBeacon
from pet_mk_iv_msgs.msg import IrRemote

class MissionNode(object):
    kLinearSpeed   = 0.4 # m/sec
    kRotationSpeed = 4.0 # rad/sec

    kForwardDistance = 0.30   # 300mm
    kSideDistance    = 0.15   # 150mm

    def __init__(self):
        rospy.init_node("mission_impossible")
        rospy.loginfo("Node Init: Starting->")

        self.is_stopped = True

        # Init Subscribers for distance/range sensors (UltraSonic)
        self.range_sensor_front_right  = -1
        self.range_sensor_middle_left = -1
        self.range_sensor_front_left   = -1
        self.range_sensor_front_right_sub  = rospy.Subscriber("range_sensor/front_right",  Range, self.callback_range_sensor_front_right )
        self.range_sensor_front_middle_sub = rospy.Subscriber("range_sensor/front_middle", Range, self.callback_range_sensor_front_middle)
        self.range_sensor_front_left_sub   = rospy.Subscriber("range_sensor/front_left",   Range, self.callback_range_sensor_front_left  )

        # Init Subscribers for Line Detection/Follower sensors
        self.line_detection_right  = None
        self.line_detection_middle = None
        self.line_detection_left   = None

        self.line_sensor_right_sub  = rospy.Subscriber("line_sensor/right",  LineDetection, self.callback_line_sensor_right )
        self.line_sensor_middle_sub = rospy.Subscriber("line_sensor/middle", LineDetection, self.callback_line_sensor_middle)
        self.line_sensor_left_sub   = rospy.Subscriber("line_sensor/left",   LineDetection, self.callback_line_sensor_left  )

        # Init Subscribe of commands from IR-Remote (via IR-sensor module)
        self.IR_sub = rospy.Subscriber("ir_remote", IrRemote, self.callback_IR_sensor)

        # Init Publisher to propulsion controller
        self.vel_pub         = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)
        
        # Init Publishers to HID (Onboard LCD-display)
        self.row1_pub        = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        self.row2_pub        = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        
        # Init publishers to HID (Onboard light beacon)
        self.beacon_mode_pub = rospy.Publisher("beacon_mode", LightBeacon, queue_size=1)

        self.vel_msg = TwistStamped()
        self.vel_msg.header.frame_id = "base_link"  # Vehicle local coordinate frame

        self.beacon_msg = LightBeacon()
        self.beacon_msg.mode = LightBeacon.ROTATING_SLOW

        rospy.wait_for_message("range_sensor/front_right",  Range, timeout=10)
        rospy.wait_for_message("range_sensor/front_middle", Range, timeout=10)
        rospy.wait_for_message("range_sensor/front_left",   Range, timeout=10)

        rospy.wait_for_message("line_sensor/right",  LineDetection, timeout=10)
        rospy.wait_for_message("line_sensor/middle", LineDetection, timeout=10)
        rospy.wait_for_message("line_sensor/left",   LineDetection, timeout=10)
    
        rospy.loginfo("Node Init: ->Done")
        return

    def start(self):
        rospy.loginfo("Node start: Starting->")

        self.row1_pub.publish("STARTED")
        self.row2_pub.publish("...tjoho!..")

        # TODO: Don't know why, but we need this sleep otherwise the beacon message is not published.
        rospy.sleep(0.5)
        self.beacon_mode_pub.publish(self.beacon_msg)

        # Create Timer for navigation logic
        self.check_for_stop_timer  = rospy.Timer(rospy.Duration(0, 100000000), self.check_for_stop)
        self.avoid_obstacles_timer = rospy.Timer(rospy.Duration(0, 100000000), self.avoid_obstacles)

        # Next expected action is "Press Play on the remote IR-controller"
        self.start_handler() # <- To use this script WITHOUT IR-remote... Then uncomment this row

        rospy.loginfo("Node start: ->Done")
        return

    # Call via "check_for_stop_timer  = rospy.Timer()"
    def check_for_stop(self, msg):
        # Stop criteria #1: Any line detected -> STOP!
        # TODO: Parametric linesAre=LineDetection.DARK or linesAre=LineDetection.LIGHT
        if (   self.line_detection_left  ==LineDetection.DARK
             or self.line_detection_middle==LineDetection.DARK
             or self.line_detection_right ==LineDetection.DARK
           ):
            rospy.logwarn("STOP! <= Line detected")
            self.row1_pub.publish("STOP")
            self.stop_handler()

        # Stop criteria #2: All distance sensors to close to obstacle -> STOP!
        elif (0 < self.range_sensor_middle_left < MissionNode.kForwardDistance and
            0 < self.range_sensor_front_left   < MissionNode.kSideDistance and
            0 < self.range_sensor_front_right  < MissionNode.kSideDistance ):
            rospy.logwarn("STOP! <= Obstacles all over the place")
            self.row1_pub.publish("STOP")
            self.row2_pub.publish("..road blocked..")
            self.stop_handler()

        return

    # Call via "avoid_obstacles_timer  = rospy.Timer()"
    def avoid_obstacles(self, msg):
        rospy.logdebug("Avoid_obstacles: Start->")
        if (self.is_stopped):
            rospy.loginfo("Avoid_obstacles: IsStoped")
            self.vel_msg.twist.linear.x  = 0.0 # STOP! No propulsion
            self.vel_msg.twist.angular.z = 0.0 # STOP! No twist/turn

        elif (0 < self.range_sensor_middle_left < MissionNode.kForwardDistance):
            rospy.loginfo("Avoid_obstacles: Obstacle detected - Straight ahead")
            self.row2_pub.publish("Obstacle-Front")
            #Obstacle straight ahead => Turn Left or Right?
            self.vel_msg.twist.linear.x = 0.0

            if (self.range_sensor_front_left > self.range_sensor_front_right):
                self.vel_msg.twist.angular.z = self.kRotationSpeed  #Turn Left CCW
            else:
                self.vel_msg.twist.angular.z = -self.kRotationSpeed #Turn Right CW

        elif (0 < self.range_sensor_front_left < MissionNode.kSideDistance):
            rospy.loginfo("Avoid_obstacles: Obstacle detected - To the Left")
            self.row2_pub.publish("Obstacle-LEFT")
            self.vel_msg.twist.linear.x = 0.0
            self.vel_msg.twist.angular.z = -self.kRotationSpeed #Turn Right CW


        elif (0 < self.range_sensor_front_right < MissionNode.kSideDistance ):
            rospy.loginfo("Avoid_obstacles: Obstacle detected - To the Right")
            self.row2_pub.publish("Obstacle-RIGHT")
            self.vel_msg.twist.linear.x = 0.0
            self.vel_msg.twist.angular.z = self.kRotationSpeed # Turn Left CCW

        else:
            rospy.loginfo("Avoid_obstacles: Run Forrest... RUN!")
            self.row2_pub.publish("Run Forrest,RUN!")
            self.vel_msg.twist.linear.x = self.kLinearSpeed
            self.vel_msg.twist.angular.z = 0.0

        self.vel_msg.header.stamp = rospy.Time.now()  # Need to set timestamp in message header before publish.
        self.vel_pub.publish(self.vel_msg)
        rospy.logdebug("Avoid_obstacles: ->Done")
        return

    def start_handler(self):
        self.is_stopped = False
        self.beacon_msg.mode = LightBeacon.ROTATING_FAST
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("start_handler: Done")

    def stop_handler(self):
        self.is_stopped = True
        self.beacon_msg.mode = LightBeacon.ROTATING_SLOW
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("stop_handler - Done")

    def abort_handler(self):
        self.is_stopped = True
        self.beacon_msg.mode = LightBeacon.OFF
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("IR-remote STOP - Abort mission script")
        self.row1_pub.publish("IR-remote STOP") 
        self.row2_pub.publish("Abort mission")            
        rospy.signal_shutdown("Mission script aborted")

    # Callback Range sensors
    def callback_range_sensor_front_right(self, msg):
        self.range_sensor_front_right = msg.range # m

    def callback_range_sensor_front_middle(self, msg):
        self.range_sensor_middle_left = msg.range # m

    def callback_range_sensor_front_left(self, msg):
        self.range_sensor_front_left = msg.range # m

    # Callback LineDetection sensors
    def callback_line_sensor_right(self, msg):
        self.line_detection_right = msg.value

    def callback_line_sensor_middle(self, msg):
        self.line_detection_middle = msg.value

    def callback_line_sensor_left(self, msg):
        self.line_detection_left = msg.value

    # Callback IR-Receiver sensor
    def callback_IR_sensor(self, msg):
        if (msg.key == IrRemote.PAUSE):
            self.stop_handler()
        elif (msg.key == IrRemote.PLAY):
            self.start_handler()
        elif (msg.key == IrRemote.STOP):
            self.abort_handler()


if __name__ == '__main__':
    rospy.loginfo("Entering main...")
    mission_impossible = MissionNode()
    mission_impossible.start()
    rospy.spin()