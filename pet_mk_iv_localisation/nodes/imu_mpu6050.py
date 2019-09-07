#!/usr/bin/env python

from __future__ import division

import smbus

import rospy

from sensor_msgs.msg import Imu

class ImuMpu6050(object):

    I2C_ADRESS = 0x68   # Found by i2cdetect command

    PWR_MGMT_1 = 0x6b

    ACCEL_XOUT = 0x3B
    ACCEL_YOUT = 0x3D
    ACCEL_ZOUT = 0x3F
    GYRO_XOUT  = 0x43
    GYRO_YOUT  = 0x45
    GYRO_ZOUT  = 0x47

    def __init__(self):
        rospy.init_node("imu")
        self._publish_rate = rospy.Rate(40)

        # Initialise hardware
        self._init_imu()

        # Publishers
        self._imu_pub = rospy.Publisher("imu", Imu, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            # Read data from imu
            linear_acc, angular_vel = self._read_data()

            # Create msg
            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "imu_frame"

            msg.linear_acceleration.x = linear_acc[0]
            msg.linear_acceleration.y = linear_acc[1]
            msg.linear_acceleration.z = linear_acc[2]

            msg.angular_velocity.x = angular_vel[0]
            msg.angular_velocity.y = angular_vel[1]
            msg.angular_velocity.z = angular_vel[2]

            msg.orientation_covariance[0] = -1  # Declare that we don't use orientation 

            # Publish msg
            self._imu_pub.publish(msg)

            self._publish_rate.sleep()

    def _init_imu(self):
        self._bus = smbus.SMBus(1)
        self._bus.write_byte_data(self.I2C_ADRESS, self.PWR_MGMT_1, 0)

    def _read_data(self):
        linear_acc = [0]*3
        linear_acc[0] = self._read_word(self.ACCEL_XOUT)
        linear_acc[1] = self._read_word(self.ACCEL_YOUT)
        linear_acc[2] = self._read_word(self.ACCEL_ZOUT)

        angular_vel = [0]*3
        angular_vel[0] = self._read_word(self.GYRO_XOUT)
        angular_vel[1] = self._read_word(self.GYRO_YOUT)
        angular_vel[2] = self._read_word(self.GYRO_ZOUT)

        return linear_acc, angular_vel

    def _read_word(self, address): 
        high = self._bus.read_byte_data(self.I2C_ADRESS, address)
        low = self._bus.read_byte_data(self.I2C_ADRESS, address+1)
        # Combine high and low byte to uint_16.
        value = (high << 8) + low
        # Return value as int_16
        if value >= 0x8000:
            return -(0xFFFF - value + 1)
        else:
            return value


if __name__ == "__main__":
    imu_handler = ImuMpu6050()
    imu_handler.run()