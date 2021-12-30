#!/usr/bin/env python
# Behaviour:
# 1) IR-remote [Play], [Pause] and [Stop] overroule them all
# 2) Follow a line on the floor. By swaying back and forth...
# 3a) Rule "a": ALL sensors detecting line -> STOP
# 3b) Rule "b": ANY distance sensors to close to obstacle => STOP (e.g. Ultrasonic HC-SR04)
# 4) Display selected action on LCD-display.
#
# Gazebo simulation launch sequence:
# 1. $ roslaunch pet_mk_iv_simulation pet_play_yard-03.launch
# 2. $ roslaunch pet_mk_iv_simulation spawn_pet_mk_iv.launch
# 3. $ rosrun pet_mk_iv_mission_control testrun_03_followline.py 
#
# IRL launch sequence:
# 1. $ roslaunch pet_mk_iv_launch pet_mk_iv.launch
# 2. $ rosrun pet_mk_iv_mission_control testrun_03_followline.py
#
# /TODO: Parametric linesAre=LineDetection.DARK or linesAre=LineDetection.LIGHT
#
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

    kForwardDistance = 0.10   # Unit = "m"
    kSideDistance    = 0.10   # Unit = "m"

    def __init__(self):
        rospy.init_node("mission_impossible")
        rospy.loginfo("Node Init: Starting->")

        self.is_stopped = True

        # Init Subscribers for distance/range sensors (UltraSonic)
        self.range_sensor_front_right  = -1
        self.range_sensor_front_middle = -1
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
        self.vel_pub   = rospy.Publisher("cmd_vel", TwistStamped, queue_size=10)
        
        # Init Publishers to HID (Onboard LCD-display)
        self.row1_pub  = rospy.Publisher("lcd_display/row1", String, queue_size=10)
        self.row2_pub  = rospy.Publisher("lcd_display/row2", String, queue_size=10)
        
        # Init publishers to HID (Onboard light beacon)
        self.beacon_mode_pub = rospy.Publisher("beacon_mode", LightBeacon, queue_size=1)

        self.vel_msg = TwistStamped()
        self.vel_msg.header.frame_id = "base_link"  # Vehicle local coordinate frame

        self.beacon_msg = LightBeacon()
        self.beacon_msg.mode = LightBeacon.ROTATING_SLOW

        rospy.wait_for_message("range_sensor/front_right",  Range)
        rospy.wait_for_message("range_sensor/front_middle", Range)
        rospy.wait_for_message("range_sensor/front_left",   Range)

        rospy.wait_for_message("line_sensor/right",  LineDetection)
        rospy.wait_for_message("line_sensor/middle", LineDetection)
        rospy.wait_for_message("line_sensor/left",   LineDetection)

        # Wait For Messages - Not workig when starting Gazebo-simulation in "pause"
        # rospy.wait_for_message("range_sensor/front_right",  Range, timeout=10)
        # rospy.wait_for_message("range_sensor/front_middle", Range, timeout=10)
        # rospy.wait_for_message("range_sensor/front_left",   Range, timeout=10)

        # rospy.wait_for_message("line_sensor/right",  LineDetection, timeout=10)
        # rospy.wait_for_message("line_sensor/middle", LineDetection, timeout=10)
        # rospy.wait_for_message("line_sensor/left",   LineDetection, timeout=10)
    
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
        self.avoid_obstacles_timer = rospy.Timer(rospy.Duration(0, 100000000), self.follow_line)

        # Next expected action is "Press Play on the remote IR-controller"
        self.start_handler() # <- Uncomment this row when running script WITHOUT IR-remote.

        rospy.loginfo("Node start: ->Done")
        return

    # Call via "check_for_stop_timer  = rospy.Timer()"
    def check_for_stop(self, msg):
        # Stop criteria/behaviour #1: ALL sensors detecting line -> STOP!
        if  (    self.line_detection_left   == LineDetection.DARK
             and self.line_detection_middle == LineDetection.DARK
             and self.line_detection_right  == LineDetection.DARK
            ):
            rospy.logwarn("follow_line > STOP - Stop line detected!")
            self.row1_pub.publish("STOP")
            self.stop_handler()

        # Stop criteria/behaviour #2: ANY distance sensors to close to obstacle -> STOP!
        elif (
                 0 < self.range_sensor_front_left   < MissionNode.kSideDistance
              or 0 < self.range_sensor_front_middle < MissionNode.kForwardDistance 
              or 0 < self.range_sensor_front_right  < MissionNode.kSideDistance
             ):
            rospy.logwarn("follow_line > STOP - Obstacles detected")
            self.row1_pub.publish("STOP")
            self.row2_pub.publish("..road blocked..")
            self.stop_handler()

        return

    # Call via "avoid_obstacles_timer  = rospy.Timer()"
    def follow_line(self, msg):
        rospy.logdebug("follow_line > Started->")
        if (self.is_stopped):
            rospy.loginfo("follow_line > IsStoped")
            self.vel_msg.twist.linear.x  = 0.0 # STOP! No propulsion
            self.vel_msg.twist.angular.z = 0.0 # STOP! No twist/turn
 
        elif(    self.line_detection_left   == LineDetection.LIGHT
             and self.line_detection_middle == LineDetection.DARK
             and self.line_detection_right  == LineDetection.LIGHT
            ):
            rospy.loginfo("follow_line > L,D,L = On Track!")
            self.row2_pub.publish("On Track!")
            self.vel_msg.twist.linear.x = self.kLinearSpeed
            self.vel_msg.twist.angular.z = 0.0

        elif(    self.line_detection_left   == LineDetection.LIGHT
             and self.line_detection_middle == LineDetection.LIGHT
             and self.line_detection_right  == LineDetection.DARK
            ):
            rospy.loginfo("follow_line > L,L,D = Turn Right!")
            self.row2_pub.publish("Turn Right")
            self.vel_msg.twist.linear.x  = self.kLinearSpeed
            self.vel_msg.twist.angular.z = -self.kRotationSpeed #Turn Right CW

        elif(    self.line_detection_left   == LineDetection.DARK
             and self.line_detection_middle == LineDetection.LIGHT
             and self.line_detection_right  == LineDetection.LIGHT
            ):
            rospy.loginfo("follow_line > D,L,L = Turn Left!")
            self.row2_pub.publish("Turn Left ")
            self.vel_msg.twist.linear.x  = self.kLinearSpeed
            self.vel_msg.twist.angular.z = self.kRotationSpeed # Turn Left CCW

        else:
            rospy.loginfo("follow_line > L,L,L = No line detected")
            self.row2_pub.publish("No line detected")
            self.vel_msg.twist.linear.x = self.kLinearSpeed
            self.vel_msg.twist.angular.z = 0.0

        self.vel_msg.header.stamp = rospy.Time.now()  # Need to set timestamp in message header before publish.
        self.vel_pub.publish(self.vel_msg)
        rospy.logdebug("follow_line > Done")
        return

    def start_handler(self):
        self.is_stopped = False
        self.beacon_msg.mode = LightBeacon.ROTATING_FAST
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("follow_line > start_handler: Done")

    def stop_handler(self):
        self.is_stopped = True
        self.beacon_msg.mode = LightBeacon.ROTATING_SLOW
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("follow_line > stop_handler = Done")

    def abort_handler(self):
        self.is_stopped = True
        self.beacon_msg.mode = LightBeacon.OFF
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.loginfo("follow_line > IR-remote STOP = Abort mission script")
        self.row1_pub.publish("IR-remote STOP") 
        self.row2_pub.publish("Abort mission")            
        rospy.signal_shutdown("Mission script aborted")

    # Callback Range sensors
    def callback_range_sensor_front_right(self, msg):
        self.range_sensor_front_right = msg.range # m

    def callback_range_sensor_front_middle(self, msg):
        self.range_sensor_front_middle = msg.range # m

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