#ifndef PET_CONTROL_KINEMATIC_MODEL_H
#define PET_CONTROL_KINEMATIC_MODEL_H

#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

class KinematicModel
{
public:
    template<typename Scalar>
    static Pose2D<Scalar> propagate(const Pose2D<Scalar>& state, const typename Pose2D<Scalar>::TangentType& twist, double dt)
    {
        return oplus<Scalar>(state, twist*dt);
    }
};

} // namespace pet::control

#endif // PET_CONTROL_KINEMATIC_MODEL_H
