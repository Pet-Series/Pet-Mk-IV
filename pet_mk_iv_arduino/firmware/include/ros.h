#ifndef _PET_ROS_H_
#define _PET_ROS_H_

#include <ros/node_handle.h>
#include <ArduinoHardware.h>

namespace pet
{
namespace ros
{
  using NodeHandle = ::ros::NodeHandle_<ArduinoHardware, 8, 8, 128, 128>;
}
}

#endif
