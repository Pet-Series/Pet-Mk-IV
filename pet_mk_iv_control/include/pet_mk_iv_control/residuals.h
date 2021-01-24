#ifndef PET_CONTROL_RESIDUALS_H
#define PET_CONTROL_RESIDUALS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/parameterization.h"

namespace pet::mpc
{

class ReferencePathResidual
{
public:
    template<typename T>
    bool operator()(const T* reference_quat_raw, const T* reference_pos_raw,
                    const T* path_quat_raw, const T* path_pos_raw,
                    T* residual_raw) const
    {
        const Pose<T> reference = Pose<T>{Eigen::Quaternion<T>{reference_quat_raw}.toRotationMatrix(), Eigen::Matrix<T,3,1>{reference_pos_raw}};
        const Pose<T> state = Pose<T>{Eigen::Quaternion<T>{path_quat_raw}.toRotationMatrix(), Eigen::Matrix<T,3,1>{path_pos_raw}};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};
        residual = ominus<T>(reference, state);
        return true;
    }

    static ceres::CostFunction* Create()
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                ReferencePathResidual, kResidualSize, kQuaternionSize, kPositionSize, kQuaternionSize, kPositionSize>;
        return new AutoDiffCostFunction(new ReferencePathResidual());
    }

private:
    static constexpr int kResidualSize = 6;
    static constexpr int kQuaternionSize = 4;
    static constexpr int kPositionSize = 3;
};

class VelocityChangeResidual
{
public:
    template<typename T>
    bool operator()(const T* velocity_current_raw,
                    const T* velocity_previous_raw,
                    T* residual_raw) const
    {
        const Eigen::Map<const Eigen::Matrix<T,kVelocitySize,1>> velocity_current{velocity_current_raw};
        const Eigen::Map<const Eigen::Matrix<T,kVelocitySize,1>> velocity_previous{velocity_previous_raw};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};
        residual = velocity_current - velocity_previous;
        return true;
    }

    static ceres::CostFunction* Create()
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                VelocityChangeResidual, kResidualSize, kVelocitySize, kVelocitySize>;
        return new AutoDiffCostFunction(new VelocityChangeResidual());
    }

private:
    static constexpr int kResidualSize = 6;
    static constexpr int kVelocitySize = 6;
};

class KinematicConstraintPenaltyResidual
{
public:
    KinematicConstraintPenaltyResidual(double dt)
        : m_dt(dt)
    {
    }

    template<typename T>
    bool operator()(const T* quat_current_raw, const T* pos_current_raw,
                    const T* quat_previous_raw, const T* pos_previous_raw,
                    const T* velocity_previous_raw,
                    T* residual_raw) const
    {
        const Pose<T> current_pose  = Pose<T>{Eigen::Quaternion<T>{quat_current_raw}.toRotationMatrix(), Eigen::Matrix<T,3,1>{pos_current_raw}};
        const Pose<T> previous_pose = Pose<T>{Eigen::Quaternion<T>{quat_previous_raw}.toRotationMatrix(), Eigen::Matrix<T,3,1>{pos_previous_raw}};
        const Eigen::Map<const Eigen::Matrix<T,kVelocitySize,1>> velocity_previous{velocity_previous_raw};
        Eigen::Map<Eigen::Matrix<T,kResidualSize,1>> residual{residual_raw};

        const Pose<T> propagated_pose = KinematicModel::propagate<T>(previous_pose, velocity_previous, m_dt);
        residual = ominus<T>(current_pose, propagated_pose);

        return true;
    }

    static ceres::CostFunction* Create(double dt)
    {
        using AutoDiffCostFunction = ceres::AutoDiffCostFunction<
                KinematicConstraintPenaltyResidual, kResidualSize, kQuaternionSize, kPositionSize, kQuaternionSize, kPositionSize, kVelocitySize>;
        return new AutoDiffCostFunction(new KinematicConstraintPenaltyResidual(dt));
    }

private:
    double m_dt;

    static constexpr int kResidualSize = 6;
    static constexpr int kQuaternionSize = 4;
    static constexpr int kPositionSize = 3;
    static constexpr int kVelocitySize = 6;
};

} // namespace pet::mpc

#endif // PET_CONTROL_RESIDUALS_H
