#ifndef _PET_ROS_H_
#define _PET_ROS_H_

// #include <ros/node_handle.h>
// #include <ArduinoHardware.h>

// namespace ros
// {
//   using NodeHandle = NodeHandle_<ArduinoHardware, 8, 8, 256, 256>;
// }

#include <ros.h>

namespace pet
{
namespace ros
{
    using NodeHandle = ::ros::NodeHandle_<ArduinoHardware, 8, 8, 128, 128, ::ros::FlashReadOutBuffer_>;
}
}

#endif
