#!/usr/bin/env python
# Behaviour:
# 1) Cycle through light baecon modes forever.
from __future__ import division

import rospy

from pet_mk_iv_msgs.msg import LightBeacon

class MissionNode(object):

    FREQUENCY = 0.2
    START_MODE = LightBeacon.ROTATING_FAST

    def __init__(self):
        rospy.init_node("light_beacon_cycler")

        self.beacon_mode_pub = rospy.Publisher("beacon_mode", LightBeacon, queue_size=10)
        self.beacon_msg = LightBeacon()
        self.beacon_msg.mode = self.START_MODE

        return

    def start(self):
        rospy.loginfo("Node starting...")

        rospy.sleep(0.5)
        self.beacon_mode_pub.publish(self.beacon_msg)
        rospy.sleep(0.5)

        period = rospy.Duration.from_sec(1/self.FREQUENCY)
        self.publish_mode_timer  = rospy.Timer(period, self.publish_mode)

        return

    def publish_mode(self, timer_event):
        # Cycle through the five different modes.
        self.beacon_msg.mode = (self.beacon_msg.mode + 1) % 5
        self.beacon_mode_pub.publish(self.beacon_msg)

        return


if __name__ == '__main__':
    mission_impossible = MissionNode()
    mission_impossible.start()
    rospy.spin()