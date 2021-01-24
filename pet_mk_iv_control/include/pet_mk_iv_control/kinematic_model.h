#ifndef PET_CONTROL_KINEMATIC_MODEL_H
#define PET_CONTROL_KINEMATIC_MODEL_H

#include "pet_mk_iv_control/parameterization.h"

namespace pet::mpc
{

class KinematicModel
{
public:
    template<typename ScalarType>
    static Pose<ScalarType> propagate(const Pose<ScalarType>& state, const Eigen::Matrix<ScalarType, 6, 1>& twist, double dt)
    {
        return oplus<ScalarType>(state, twist*dt);
    }
};

}

#endif // PET_CONTROL_KINEMATIC_MODEL_H
