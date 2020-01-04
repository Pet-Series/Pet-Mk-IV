#!/usr/bin/env python

from __future__ import division

import math
import smbus

import rospy

from sensor_msgs.msg import Imu

class MPU6050(object):

    I2C_ADRESS = 0x68   # Found by i2cdetect command

    # Configuration values:
    GYRO_250_DPS    = 0 << 3
    GYRO_500_DPS    = 1 << 3
    GYRO_1000_DPS   = 2 << 3
    GYRO_2000_DPS   = 4 << 3
    ACCEL_2_G       = 0 << 3
    ACCEL_4_G       = 1 << 3
    ACCEL_8_G       = 2 << 3
    ACCEL_16_G      = 4 << 3

    # MPU6050 Registers and their Address:
    GYRO_CONFIG     = 0x1B
    ACCEL_CONFIG    = 0x1C
    PWR_MGMT_1      = 0x6B
    ACCEL_XOUT      = 0x3B
    ACCEL_YOUT      = 0x3D
    ACCEL_ZOUT      = 0x3F
    GYRO_XOUT       = 0x43
    GYRO_YOUT       = 0x45
    GYRO_ZOUT       = 0x47

    # Sensitivity values taken from table at: https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
    # GYRO_250_DPS:   131,
    # GYRO_500_DPS:   65.5,
    # GYRO_1000_DPS:  32.8,
    # GYRO_2000_DPS:  16.4,
    # ACCEL_2_G:      16384,
    # ACCEL_4_G:      8192,
    # ACCEL_8_G:      4096,
    # ACCEL_16_G:     2048,

    # Value range of +-2g put into 16-bit integer.
    LINEAR_ACC_SCALE = 9.82 / 16384

    # Value range of +-250 deg/s put into 16-bit integer. (Table says 131, why?).
    ANGULAR_VEL_SCALE = (math.pi/180) / 131

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
        self._bus.write_byte_data(self.I2C_ADRESS, MPU6050.PWR_MGMT_1, 0)
        self._bus.write_byte_data(self.I2C_ADRESS, MPU6050.GYRO_CONFIG, MPU6050.GYRO_250_DPS)
        self._bus.write_byte_data(self.I2C_ADRESS, MPU6050.ACCEL_CONFIG, MPU6050.ACCEL_2_G)

    def _read_data(self):
        linear_acc = [0]*3
        linear_acc[0] = self._read_word(MPU6050.ACCEL_XOUT) * MPU6050.LINEAR_ACC_SCALE
        linear_acc[1] = self._read_word(MPU6050.ACCEL_YOUT) * MPU6050.LINEAR_ACC_SCALE
        linear_acc[2] = self._read_word(MPU6050.ACCEL_ZOUT) * MPU6050.LINEAR_ACC_SCALE

        angular_vel = [0]*3
        angular_vel[0] = self._read_word(MPU6050.GYRO_XOUT) * MPU6050.ANGULAR_VEL_SCALE
        angular_vel[1] = self._read_word(MPU6050.GYRO_YOUT) * MPU6050.ANGULAR_VEL_SCALE
        angular_vel[2] = self._read_word(MPU6050.GYRO_ZOUT) * MPU6050.ANGULAR_VEL_SCALE

        return linear_acc, angular_vel

    def _read_word(self, register): 
        high = self._bus.read_byte_data(self.I2C_ADRESS, register)
        low = self._bus.read_byte_data(self.I2C_ADRESS, register+1)
        # Combine high and low byte to uint_16.
        value = (high << 8) + low
        # Return value as int_16
        if value >= 0x8000:
            return -(0xFFFF - value + 1)
        else:
            return value


if __name__ == "__main__":
    imu_handler = MPU6050()
    imu_handler.run()