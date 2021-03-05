#ifndef PET_CONTROL_SETPOINT_H
#define PET_CONTROL_SETPOINT_H

#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

struct Setpoint
{
    double time_stamp;
    Pose2D<double> pose;

    Setpoint() = default;
    Setpoint(double t_time_stamp, Pose2D<double> t_pose);
};

inline
bool operator<(const Setpoint& lhs, const Setpoint& rhs)
{
    return lhs.time_stamp < rhs.time_stamp;
}

} // namespace pet::control

#endif // PET_CONTROL_SETPOINT_H
