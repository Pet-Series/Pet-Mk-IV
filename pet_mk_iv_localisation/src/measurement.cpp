#include "measurement.h"

#include <ros/time.h>

namespace pet
{

Measurement::Measurement(const ros::Time& stamp)
    : m_stamp(stamp)
{
}

} // namespace pet
