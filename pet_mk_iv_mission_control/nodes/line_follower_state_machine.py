#!/usr/bin/env python

from __future__ import division

import rospy
# <Turn Left> "Line to the Left"
# <Forward> "On track"
# <Turn Right> "Line to the Right"
# <Stop> "Finish"
# <Stop> "Undefined"

# bool left
# bool middle
# bool right
from pet_mk_iv_msgs.msg import TripleBoolean

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard Left={}, Middle={}, Right={}".format(msg.left,msg.middle,msg.right))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("line_followers", TripleBoolean, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

