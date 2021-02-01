#ifndef PET_CONTROL_RESIDUALS_H
#define PET_CONTROL_RESIDUALS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/parameterization2d.h"

namespace pet::control
{

class ReferencePathResidual
{
public:
    template<typename T>
    bool operator()(const T* reference_rot_raw, const T* reference_pos_raw,
                    const T* path_rot_raw, const T* path_pos_raw,
                    T* residual_raw) const
    {
        const Pose2D<T> reference = Pose2D<T>{typename Pose2D<T>::RotationType{reference_rot_raw}, typename Pose2D<T>::PointType{reference_pos_raw}};
        const Pose2D<T> state = Pose2D<T>{typename Pose2D<T>::RotationType{path_rot_raw}, typename Pose2D<T>::PointType{path_pos_raw}};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};
        residual = ominus<T>(reference, state);
        return true;
    }

    static ceres::CostFunction* Create()
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                ReferencePathResidual, kResidualSize, kRotationSize, kPositionSize, kRotationSize, kPositionSize>;
        return new AutoDiffCostFunction(new ReferencePathResidual());
    }

private:
    static constexpr int kResidualSize = 3;
    static constexpr int kRotationSize = 4;
    static constexpr int kPositionSize = 2;
};

class VelocityChangeResidual
{
public:
    template<typename T>
    bool operator()(const T* twist_current_raw,
                    const T* twist_previous_raw,
                    T* residual_raw) const
    {
        const Eigen::Map<const Eigen::Matrix<T,kTwistSize,1>> twist_current{twist_current_raw};
        const Eigen::Map<const Eigen::Matrix<T,kTwistSize,1>> twist_previous{twist_previous_raw};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};
        residual = twist_current - twist_previous;
        return true;
    }

    static ceres::CostFunction* Create()
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                VelocityChangeResidual, kResidualSize, kTwistSize, kTwistSize>;
        return new AutoDiffCostFunction(new VelocityChangeResidual());
    }

private:
    static constexpr int kResidualSize = 3;
    static constexpr int kTwistSize = 3;
};

class KinematicConstraintPenaltyResidual
{
public:
    KinematicConstraintPenaltyResidual(double dt)
        : m_dt(dt)
    {
    }

    template<typename T>
    bool operator()(const T* rot_current_raw, const T* pos_current_raw,
                    const T* rot_previous_raw, const T* pos_previous_raw,
                    const T* twist_previous_raw,
                    T* residual_raw) const
    {
        const Pose2D<T> current_pose  = Pose2D<T>{typename Pose2D<T>::RotationType{rot_current_raw}, typename Pose2D<T>::PointType{pos_current_raw}};
        const Pose2D<T> previous_pose = Pose2D<T>{typename Pose2D<T>::RotationType{rot_previous_raw}, typename Pose2D<T>::PointType{pos_previous_raw}};
        const Eigen::Map<const Eigen::Matrix<T,kTwistSize,1>> twist_previous{twist_previous_raw};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};

        const Pose2D<T> propagated_pose = KinematicModel::propagate<T>(previous_pose, twist_previous, m_dt);
        residual = ominus<T>(current_pose, propagated_pose);

        return true;
    }

    static ceres::CostFunction* Create(double dt)
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                KinematicConstraintPenaltyResidual, kResidualSize, kRotationSize, kPositionSize, kRotationSize, kPositionSize, kTwistSize>;
        return new AutoDiffCostFunction(new KinematicConstraintPenaltyResidual(dt));
    }

private:
    double m_dt;

    static constexpr int kResidualSize = 3;
    static constexpr int kRotationSize = 4;
    static constexpr int kPositionSize = 2;
    static constexpr int kTwistSize = 3;
};

} // namespace pet::control

#endif // PET_CONTROL_RESIDUALS_H
