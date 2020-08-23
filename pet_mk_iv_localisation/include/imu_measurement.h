#ifndef PET_LOCALISATION_IMU_MEASUREMENT_H
#define PET_LOCALISATION_IMU_MEASUREMENT_H

#include <sensor_msgs/Imu.h>

#include <ugl/math/vector.h>

#include "measurement.h"

namespace pet
{

class ImuMeasurement: public Measurement
{
public:
    ImuMeasurement(const sensor_msgs::Imu& imu_msg);

    // Acceleration as measured by accelerometer.
    const ugl::Vector3& acceleration() const;

    // Angular rate as measured by gyroscope.
    const ugl::Vector3& angular_rate() const;

private:
    ugl::Vector3 m_acc;
    ugl::Vector3 m_rate;
};

} // namespace pet

#endif // PET_LOCALISATION_IMU_MEASUREMENT_H