#ifndef PET_CONTROL_ROTATION2D_H
#define PET_CONTROL_ROTATION2D_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pet::mpc
{

template<typename Scalar>
struct SO2
{
    using MatrixType = Eigen::Matrix<Scalar, 2, 2>;
    using TangentType = Eigen::Matrix<Scalar, 1, 1>;

    static MatrixType hat(const Scalar& phi)
    {
        MatrixType matrix = MatrixType::Zero();
        matrix(0,1) = -phi;
        matrix(1,0) =  phi;
        return matrix;
    }

    static Scalar vee(const MatrixType& matrix)
    {
        return 0.5 * (matrix(1,0) - matrix(0,1));
    }

    static MatrixType exp(const TangentType& phi)
    {
        return exp(phi(0));
    }

    static MatrixType exp(const Scalar& phi)
    {
        return MatrixType::Identity() * cos(phi) + SO2<double>::hat(1.0) * sin(phi);
    }

    static Scalar log(const MatrixType& R)
    {
        return asin(0.5 * vee(R - R.transpose()));
    }

    static MatrixType left_jacobian(const TangentType& phi)
    {
        return left_jacobian(phi(0));
    }

    static MatrixType left_jacobian(const Scalar& phi)
    {
        constexpr double kTolerance = 1e-10;
        if (phi < kTolerance) {
            return MatrixType::Identity() + 0.5 * hat(phi);
        }
        return MatrixType::Identity() * sin(phi) / phi + SO2<double>::hat(1.0) * (1.0 - cos(phi)) / phi;
    }

    static MatrixType left_jacobian_inv(const TangentType& phi)
    {
        return left_jacobian_inv(phi(0));
    }

    static MatrixType left_jacobian_inv(const Scalar& phi)
    {
        constexpr double kTolerance = 1e-10;
        if (phi < kTolerance) {
            return MatrixType::Identity() - 0.5 * hat(phi);
        }
        return 0.5 * phi*phi / (1.0 - cos(phi)) * left_jacobian(phi).transpose();
    }

    static Eigen::Quaternion<Scalar> to_quaternion(const MatrixType& R)
    {
        const auto angle = log(R);
        const auto axis = Eigen::Matrix<Scalar, 3, 1>::UnitZ();
        return Eigen::Quaternion<Scalar>{Eigen::AngleAxisd{angle, axis}};
    }

    static MatrixType from_quaternion(const Eigen::Quaternion<Scalar>& q)
    {
        /// TODO: Assert that q only rotates around z-axis.
        return exp(2.0 * acos(q.w()));
    }
};

} // namespace pet::mpc

#endif // PET_CONTROL_ROTATION2D_H
