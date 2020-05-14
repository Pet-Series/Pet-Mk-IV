#!/usr/bin/env python

from __future__ import division

import rospy

from geometry_msgs.msg import TwistStamped

from pet_mk_iv_msgs.msg import EngineCommand

class Controller(object):

    width = 0.088
    pwm_vel_ratio = 255 / 0.52  # Velocity measured with PWM=128 -> 0.26 m/s.


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
                
                linear_vel = self._vel_msg.twist.linear.x
                angular_vel = self._vel_msg.twist.angular.z

                left_wheel, right_wheel = self.to_wheel_vel(linear_vel, angular_vel)
                msg.left_pwm = self.vel_to_pwm(left_wheel)
                msg.right_pwm = self.vel_to_pwm(right_wheel)

                msg.left_direction = sign(left_wheel)
                msg.right_direction = sign(right_wheel)

                self._engine_pub.publish(msg)
                self._vel_msg = None

            self._cmd_rate.sleep()

    def _vel_cb(self, msg):
        self._vel_msg = msg

    def to_wheel_vel(self, linear_vel, angular_vel):
        """Converts desired linear and angular velocity to desired velocity of left and right wheel."""
        left_wheel = linear_vel - angular_vel * self.width/2
        right_wheel = linear_vel + angular_vel * self.width/2
        return left_wheel, right_wheel

    def vel_to_pwm(self, vel):
        """Converts velocity to PWM value [0,255]. Ignores signedness of given velocity."""
        return int(min(abs(vel * self.pwm_vel_ratio), 255))

def sign(x):
    """Returns (+-)1 with same sign as x (or 0 if x == 0)."""
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


if __name__ == "__main__":
    controller = Controller()
    controller.run()
