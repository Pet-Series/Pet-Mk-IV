#include "sonar_measurement.h"

#include <pet_mk_iv_msgs/DistanceMeasurement.h>

#include "measurement.h"

namespace pet
{

SonarMeasurement::SonarMeasurement(const pet_mk_iv_msgs::DistanceMeasurement& sonar_msg)
    : Measurement(sonar_msg.header.stamp)
    , m_distance(sonar_msg.distance * 1000.0)
{
}

} // namespace pet
