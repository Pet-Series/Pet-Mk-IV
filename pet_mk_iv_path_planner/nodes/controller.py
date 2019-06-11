#!/usr/bin/env python

from __future__ import division

import rospy

from pet_mk_iv_msgs.msg import EngineCommand

class Controller(object):

    def __init__(self):
        rospy.init_node("controller")

        engine_pub = rospy.Publisher("engine_command", EngineCommand, queue_size=2)

        cmd_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg = EngineCommand()
            msg.header.stamp = rospy.Time.now()
            msg.left_direction = EngineCommand.FORWARD
            msg.right_direction = EngineCommand.FORWARD
            msg.left_pwm = 255
            msg.right_pwm = 255

            engine_pub.publish(msg)

            cmd_rate.sleep()


if __name__ == "__main__":
    controller = Controller()
