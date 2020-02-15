#ifndef _ROS_TIME_OP_H
#define _ROS_TIME_OP_H

#include <ros/time.h>

namespace ros
{
    bool operator<(const Time& lhs, const Time& rhs);
    Time operator+(const Time& lhs, const Duration rhs);
    Time operator-(const Time& lhs, const Duration rhs);
}

#endif // _ROS_TIME_OP_H