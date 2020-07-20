#!/usr/bin/env python

from __future__ import division

import math

import rospy
import tf2_ros

from tf.transformations import quaternion_from_euler

import geometry_msgs.msg as geo_msgs
from sensor_msgs.msg import Imu

class InertialIntegrator(object):

    def __init__(self):
        rospy.init_node("inertial_integrator")

        self.freq       = rospy.get_param("~frequency", default=20)
        self.map_frame  = rospy.get_param("~map_frame", default="map")
        self.body_frame = rospy.get_param("~body_frame", default="base_link")

        self.tf_buffer      = tf2_ros.Buffer()
        self.tf_listener    = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.tf_msg = geo_msgs.TransformStamped()
        self.tf_msg.header.frame_id = self.map_fram
        self.tf_msg.child_frame_id = self.body_frame

        self.imu_msg = None
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_cb)
        rospy.wait_for_message(self.imu_sub.topic, self.imu_sub.topic_type, timeout=10)

        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0

        return

    def imu_cb(self, msg: Imu):
        self.imu_msg = msg

    def start(self):
        self.measure_static_bias()

        self.timer = rospy.Timer(rospy.Duration.from_sec(1/self.freq), self.timer_cb)

        return

    def measure_static_bias(self):

        return

    def timer_cb(self, event: rospy.TimerEvent):
        if self.imu_msg is None:
            return

        map_imu_msg = self.tf_buffer.transform(self.imu_msg, self.map_frame, timeout=rospy.Duration(0.1))

        dt = 1 / self.freq
        ax = map_imu_msg.linear_acceleration.x
        ay = map_imu_msg.linear_acceleration.y
        wz = map_imu_msg.angular_velocity.z

        self.x += self.vx*dt + ax*0.5*dt*dt
        self.y += self.vy*dt + ay*0.5*dt*dt

        self.vx += ax*dt
        self.vy += ay*dt

        self.yaw += wz*dt 
        self.yaw = self.yaw % (2*math.pi)

        self.publish_transform()

        return

    def publish_transform(self):
        self.tf_msg.header.stamp = rospy.Time.now()
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        self.tf_msg.transform.rotation = quaternion_from_euler(0, 0, self.yaw)

        self.tf_broadcaster.sendTransform(self.tf_msg)

        return


if __name__ == "__main__":
    node = InertialIntegrator()
    node.start()