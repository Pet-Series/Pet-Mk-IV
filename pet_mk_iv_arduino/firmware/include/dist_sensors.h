#ifndef _PET_DISTSENSORS_H
#define _PET_DISTSENSORS_H

#include <ros/duration.h>

namespace dist_sensors
{

constexpr ros::Duration kPeriod(0.033);

void setup();
void callback();

} // namespace dist_sensors

#endif
