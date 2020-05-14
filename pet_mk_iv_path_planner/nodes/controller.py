#!/usr/bin/env python

from __future__ import division

import rospy

from geometry_msgs.msg import TwistStamped

from pet_mk_iv_msgs.msg import EngineCommand

class Controller(object):

    def __init__(self):
        rospy.init_node("controller")
        self._cmd_rate = rospy.Rate(100)

        # Subscribers
        self._vel_msg = None
        self._vel_sub = rospy.Subscriber("vel_cmd", TwistStamped, self._vel_cb)

        # Publishers
        self._engine_pub = rospy.Publisher("engine_command", EngineCommand, queue_size=2)

    def run(self):
        while not rospy.is_shutdown():
            if self._vel_msg is not None:
                msg = EngineCommand()
                msg.header.stamp = self._vel_msg.header.stamp
                msg.left_pwm = 128
                msg.right_pwm = 128

                if self._vel_msg.twist.linear.x > 0:
                    msg.left_direction = EngineCommand.FORWARD
                    msg.right_direction = EngineCommand.FORWARD
                elif self._vel_msg.twist.linear.x < 0:
                    msg.left_direction = EngineCommand.BACKWARD
                    msg.right_direction = EngineCommand.BACKWARD
                elif self._vel_msg.twist.angular.z > 0:
                    msg.left_direction = EngineCommand.BACKWARD
                    msg.right_direction = EngineCommand.FORWARD
                elif self._vel_msg.twist.angular.z < 0:
                    msg.left_direction = EngineCommand.FORWARD
                    msg.right_direction = EngineCommand.BACKWARD
                else:
                    msg.left_direction = EngineCommand.STOP
                    msg.right_direction = EngineCommand.STOP

                self._engine_pub.publish(msg)
                self._vel_msg = None

            self._cmd_rate.sleep()

    def _vel_cb(self, msg):
        self._vel_msg = msg


if __name__ == "__main__":
    controller = Controller()
    controller.run()
