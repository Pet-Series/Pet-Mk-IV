#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from pet_mk_iv_msgs.msg import LineDetection

GRAYSCALE_THRESHOLD = 50

def callback(image):
    sum = 0
    for pixel in image.data:
        sum += int(pixel.encode('hex'), base=16)
    mean = sum / len(image.data)

    line_msg = LineDetection()
    line_msg.header = image.header
    if mean > GRAYSCALE_THRESHOLD:
        line_msg.value = LineDetection.LIGHT
    else:
        line_msg.value = LineDetection.DARK

    publisher.publish(line_msg)


if __name__ == "__main__":
    rospy.init_node("line_sensor_gazebo_converter")

    publisher = rospy.Publisher("line_sensor", LineDetection, queue_size=10)
    subscriber = rospy.Subscriber("camera", Image, callback)

    rospy.spin()

