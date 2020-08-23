#ifndef PET_LOCALISATION_SONAR_MEASUREMENT_H
#define PET_LOCALISATION_SONAR_MEASUREMENT_H

#include <pet_mk_iv_msgs/DistanceMeasurement.h>

#include "measurement.h"

namespace pet
{

class SonarMeasurement: public Measurement
{
public:
    SonarMeasurement(const pet_mk_iv_msgs::DistanceMeasurement& sonar_msg);

    // Distance measured by sonar.
    double distance() const
    {
        return m_distance;
    }

private:
    double m_distance;
};

} // namespace pet

#endif // PET_LOCALISATION_SONAR_MEASUREMENT_H