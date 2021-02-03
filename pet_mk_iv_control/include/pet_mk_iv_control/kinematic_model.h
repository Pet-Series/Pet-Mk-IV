#ifndef PET_CONTROL_KINEMATIC_MODEL_H
#define PET_CONTROL_KINEMATIC_MODEL_H

#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

class KinematicModel
{
public:
    struct Parameters
    {
        double max_linear_speed = 0.5;
        double max_angular_speed = 4.0;
    };

public:
    explicit KinematicModel(const Parameters& parameters)
        : m_parameters(parameters)
    {
    }

    template<typename Scalar>
    static Pose2D<Scalar> propagate(const Pose2D<Scalar>& state, const typename Pose2D<Scalar>::TangentType& twist, double dt)
    {
        return oplus<Scalar>(state, twist*dt);
    }

    auto get_max_linear_speed() const
    {
        return m_parameters.max_linear_speed;
    }

    auto get_max_angular_speed() const
    {
        return m_parameters.max_angular_speed;
    }

private:
    Parameters m_parameters;
};

} // namespace pet::control

#endif // PET_CONTROL_KINEMATIC_MODEL_H
