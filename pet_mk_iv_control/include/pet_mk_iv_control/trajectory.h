#ifndef PET_CONTROL_TRAJECTORY_H
#define PET_CONTROL_TRAJECTORY_H

#include <vector>

#include "pet_mk_iv_control/pose2d.h"
#include "pet_mk_iv_control/setpoint.h"

namespace pet::control
{

class LinearTrajectory
{
public:
    /// @param control_points sorted list of control points
    explicit LinearTrajectory(const std::vector<Setpoint>& control_points);

    /// @brief Total duration of the trajectory.
    double duration() const;

    const auto& get_control_points() const
    {
        return m_control_points;
    }

    /// @brief Start pose of the trajectory.
    Pose2D<double> start() const;

    /// @brief End pose of the trajectory.
    Pose2D<double> end() const;

    Pose2D<double> pose(double t) const;

    Pose2D<double>::TangentType twist(double t) const;

private:
    std::vector<Setpoint> m_control_points;
};

} // namespace pet::control

#endif // PET_CONTROL_TRAJECTORY_H
