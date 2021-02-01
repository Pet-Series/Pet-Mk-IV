#ifndef PET_CONTROL_PARAMETERIZATION_2D_H
#define PET_CONTROL_PARAMETERIZATION_2D_H

#include <Eigen/Core>

#include <ceres/autodiff_local_parameterization.h>

#include "pet_mk_iv_control/rotation2d.h"
#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

struct Rotation2DPlus
{
    static constexpr int kGlobalSize = 4;
    static constexpr int kLocalSize = 1;

    template<typename Scalar>
    bool operator()(const Scalar x_raw[kGlobalSize], const Scalar delta_raw[kLocalSize], Scalar x_plus_delta_raw[kGlobalSize]) const
    {
        const Eigen::Map<const typename SO2<Scalar>::MatrixType> x{x_raw};
        const Eigen::Map<const typename SO2<Scalar>::TangentType> delta{delta_raw};
        Eigen::Map<typename SO2<Scalar>::MatrixType> x_plus_delta{x_plus_delta_raw};

        x_plus_delta = x * SO2<Scalar>::exp(delta);
        return true;
    }
};

using Rotation2DParameterization = ceres::AutoDiffLocalParameterization<Rotation2DPlus, Rotation2DPlus::kGlobalSize, Rotation2DPlus::kLocalSize>;

} // namespace pet::control

#endif // PET_CONTROL_PARAMETERIZATION_2D_H
