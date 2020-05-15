#include <ros/time.h>

#include "ros_time_operators.h"

namespace ros
{
    bool operator<(const Time& lhs, const Time& rhs)
    {
        if (lhs.sec < rhs.sec) {
            return true;
        } else if (lhs.sec > rhs.sec) {
            return false;
        } else if ( lhs.nsec < rhs.nsec) {
            return true;
        } else {
            return false;
        }
    }

    Time operator+(const Time& lhs, const Duration rhs)
    {
        Time temp = lhs;
        temp += rhs;
        return temp;
    }

    Time operator-(const Time& lhs, const Duration rhs)
    {
        Time temp = lhs;
        temp -= rhs;
        return temp;
    }
}