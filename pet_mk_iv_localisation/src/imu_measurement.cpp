#include "imu_measurement.h"

#include <sensor_msgs/Imu.h>

#include <ugl/math/vector.h>

#include <ugl_ros/convert_tf2.h>

#include "measurement.h"

namespace pet
{

ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu& imu_msg)
    : MeasurementBase(imu_msg.header.stamp)
    , m_acc(tf2::fromMsg(imu_msg.linear_acceleration))
    , m_rate(tf2::fromMsg(imu_msg.angular_velocity))
{
}

const ugl::Vector3& ImuMeasurement::acceleration() const
{
    return m_acc;
}

const ugl::Vector3& ImuMeasurement::angular_rate() const
{
    return m_rate;
}

} // namespace pet
