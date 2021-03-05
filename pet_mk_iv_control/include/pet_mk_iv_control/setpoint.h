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

} // namespace pet::control

#endif // PET_CONTROL_SETPOINT_H
