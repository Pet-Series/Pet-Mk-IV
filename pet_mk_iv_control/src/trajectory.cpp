#include "pet_mk_iv_control/trajectory.h"

#include <algorithm>
#include <iterator>
#include <vector>

#include <ros/assert.h>

#include "pet_mk_iv_control/setpoint.h"

namespace pet::control
{

LinearTrajectory::LinearTrajectory(const std::vector<Setpoint>& control_points)
    : m_control_points(control_points)
{
}

double LinearTrajectory::duration() const
{
    return m_control_points.back().time_stamp;
}

Pose2D<double> LinearTrajectory::start() const
{
    return m_control_points.front().pose;
}

Pose2D<double> LinearTrajectory::end() const
{
    return m_control_points.back().pose;
}

Pose2D<double> LinearTrajectory::pose(double t) const
{
    ROS_ASSERT_MSG(t < 0 || t > duration(), "Argument t is outside of its valid range. [t = %d]", t);

    auto it = std::find_if(
        std::begin(m_control_points), std::end(m_control_points),
        [&](const Setpoint& setpoint) { return setpoint.time_stamp > t; }
    );

    const auto& p1 = std::prev(it)->pose;
    const auto& t1 = std::prev(it)->time_stamp;
    const auto& p2 = it->pose;
    const auto& t2 = it->time_stamp;

    // Linear interpolation on Lie group.
    const double delta_t = t - t1;
    const auto twist = ominus(p2, p1) / (t2 - t1);
    return oplus(p1, delta_t * twist);
}

Pose2D<double>::TangentType LinearTrajectory::twist(double t) const
{
    ROS_ASSERT_MSG(t < 0 || t > duration(), "Argument t is outside of its valid range. [t = %d]", t);

    auto it = std::find_if(
        std::begin(m_control_points), std::end(m_control_points),
        [&](const Setpoint& setpoint) { return setpoint.time_stamp > t; }
    );

    const auto& p1 = std::prev(it)->pose;
    const auto& t1 = std::prev(it)->time_stamp;
    const auto& p2 = it->pose;
    const auto& t2 = it->time_stamp;

    return ominus(p2, p1) / (t2 - t1);
}

} // namespace pet::control
