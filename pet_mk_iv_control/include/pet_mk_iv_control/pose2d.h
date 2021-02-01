#ifndef PET_CONTROL_POSE2D_H
#define PET_CONTROL_POSE2D_H

#include <Eigen/Core>

#include "pet_mk_iv_control/rotation2d.h"

namespace pet::control
{

template<typename Scalar>
struct Pose2D
{
    using RotationType = typename SO2<Scalar>::MatrixType;
    using PointType = Eigen::Matrix<Scalar, 2, 1>;
    using MatrixType = Eigen::Matrix<Scalar, 3, 3>;
    using TangentType = Eigen::Matrix<Scalar, 3, 1>;

    RotationType rotation;
    PointType position;

    static MatrixType hat(const TangentType& tau)
    {
        MatrixType matrix = MatrixType::Zero();
        matrix.template block<2,2>(0,0) = SO2<Scalar>::hat(tau(0));
        matrix.template block<2,1>(0,2) = tau.template segment<2>(1);
        return matrix;
    }

    static Scalar vee(const MatrixType& matrix)
    {
        TangentType tau;
        tau(0) = SO2<Scalar>::vee(matrix.template block<2,2>(0,0));
        tau.template segment<2>(1) = matrix.template block<2,1>(0,2);
        return tau;
    }

    static Pose2D exp(const TangentType& tau)
    {
        const Scalar& phi = tau(0);
        const PointType& rho = tau.template segment<2>(1);
        const auto J = SO2<Scalar>::left_jacobian(phi);
        return Pose2D{SO2<Scalar>::exp(phi), J * rho};
    }

    static TangentType log(const Pose2D& pose)
    {
        const auto phi = SO2<Scalar>::log(pose.rotation);
        const auto Jinv = SO2<Scalar>::left_jacobian_inv(phi);
        const PointType rho = Jinv * pose.position;
        TangentType tau;
        tau << phi, rho;
        return tau;
    }
};

template<typename Scalar>
Pose2D<Scalar> operator*(const Pose2D<Scalar>& lhs, const Pose2D<Scalar>& rhs)
{
    Pose2D<Scalar> result;
    result.rotation = lhs.rotation * rhs.rotation;
    result.position = lhs.position + lhs.rotation * rhs.position;
    return result;
}

template<typename Scalar>
Pose2D<Scalar> inverse(const Pose2D<Scalar>& pose)
{
    auto rotation_inverse = pose.rotation.transpose();
    return Pose2D<Scalar>{rotation_inverse, -rotation_inverse*pose.position};
}

template<typename Scalar>
Pose2D<Scalar> oplus(const Pose2D<Scalar>& lhs, const typename Pose2D<Scalar>::TangentType& rhs)
{
    return lhs * Pose2D<Scalar>::exp(rhs);
}

template<typename Scalar>
typename Pose2D<Scalar>::TangentType ominus(const Pose2D<Scalar>& lhs, const Pose2D<Scalar>& rhs)
{
    return Pose2D<Scalar>::log(inverse(rhs) * lhs);
}

} // namespace pet::control

#endif // PET_CONTROL_POSE2D_H
