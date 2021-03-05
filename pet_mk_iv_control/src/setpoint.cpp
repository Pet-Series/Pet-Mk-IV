#include "pet_mk_iv_control/setpoint.h"

#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

Setpoint::Setpoint(double t_time_stamp, Pose2D<double> t_pose)
    : time_stamp(t_time_stamp)
    , pose(t_pose)
{}

} // namespace pet::control
