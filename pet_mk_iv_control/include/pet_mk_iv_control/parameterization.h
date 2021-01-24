#ifndef PET_CONTROL_PARAMETERIZATION_H
#define PET_CONTROL_PARAMETERIZATION_H

#include <Eigen/Core>

namespace pet::mpc
{

// template<typename ScalarType>
// using Pose = Eigen::Matrix<ScalarType, 4, 4>

template<typename ScalarType>
struct Pose
{
    Eigen::Matrix<ScalarType, 3, 3> rotation;
    Eigen::Matrix<ScalarType, 3, 1> position;
};

template<typename ScalarType>
Pose<ScalarType> operator*(const Pose<ScalarType>& lhs, const Pose<ScalarType>& rhs)
{
    Pose<ScalarType> res;
    res.rotation = lhs.rotation * rhs.rotation;
    res.position = lhs.position + lhs.rotation * rhs.position;
    return res;
}

template<typename ScalarType>
Pose<ScalarType> inverse(const Pose<ScalarType>& pose)
{
    auto rot_inv = pose.rotation.transpose();
    return Pose<ScalarType>{rot_inv, -rot_inv*pose.position};
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 3> SO3_hat(const Eigen::Matrix<ScalarType, 3, 1>& w)
{
    Eigen::Matrix<ScalarType, 3, 3> S;
    S(0,0) = ScalarType{0.0}; S(0,1) = -w(2); S(0,2) = w(1);
    S(1,0) = w(2); S(1,1) = ScalarType{0.0}; S(1,2) = -w(0);
    S(2,0) = -w(1); S(2,1) = w(0); S(1,2) = ScalarType{0.0};
    // S << 0.0, -w(2), w(1),
    //      w(2), 0.0, -w(0),
    //     -w(1), w(0), 0.0;
    return S;
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 1> SO3_vee(const Eigen::Matrix<ScalarType, 3, 3>& S)
{
    return 0.5 * Eigen::Matrix<ScalarType, 3, 1>{S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)};
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 3> SO3_exp(const Eigen::Matrix<ScalarType, 3, 1>& phi)
{
    constexpr double kTolerance = 1e-10;
    const ScalarType theta = phi.norm();
    if (theta < kTolerance) {
        return SO3_hat(phi);
    }
    const Eigen::Matrix<ScalarType, 3, 3> S = SO3_hat(phi);
    return Eigen::Matrix<ScalarType, 3, 3>::Identity() + sin(theta)/theta * S + ((1.0-cos(theta))/(theta*theta))*S*S;
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 1> SO3_log(const Eigen::Matrix<ScalarType, 3, 3>& R)
{
    constexpr double kTolerance = 1e-10;
    const ScalarType trace = R.trace();
    if (trace + 1.0 < kTolerance) {
        // const ScalarType R00 = R(0,0);
        // const ScalarType R11 = R(1,1);
        // const ScalarType R22 = R(2,2);
        if (abs(R(2,2) + 1.0) > kTolerance) {
            return M_PI / sqrt(2.0 * (R(2,2) + 1.0)) * (R.col(2) + Eigen::Matrix<ScalarType, 3, 1>::UnitZ());
        }
        else if (abs(R(1,1) + 1.0) > kTolerance) {
            return M_PI / sqrt(2.0 * (R(1,1) + 1.0)) * (R.col(1) + Eigen::Matrix<ScalarType, 3, 1>::UnitY());
        }
        else /* if(abs(R(0,0) + 1.0) > kTolerance) */ {
            return M_PI / sqrt(2.0 * (R(0,0) + 1.0)) * (R.col(0) + Eigen::Matrix<ScalarType, 3, 1>::UnitX());
        }
    }
    const ScalarType theta = acos(0.5 * (trace - 1.0));
    if (theta < kTolerance) {
        // return SO3_vee<ScalarType>(R - Eigen::Matrix<ScalarType, 3, 3>::Identity());
        return SO3_vee<ScalarType>(R - R.transpose());
    }
    return 0.5 * theta / sin(theta) * SO3_vee<ScalarType>(R - R.transpose());
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 3> SO3_left_jacobian(const Eigen::Matrix<ScalarType, 3, 1>& phi)
{
    constexpr double kTolerance = 1e-10;
    const ScalarType phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return Eigen::Matrix<ScalarType, 3, 3>::Identity() + 0.5 * SO3_hat(phi);
    }
    const Eigen::Matrix<ScalarType, 3, 3> W = SO3_hat(phi) / phi_norm;
    return Eigen::Matrix<ScalarType, 3, 3>::Identity() + ((1.0 - cos(phi_norm)) / phi_norm) * W + ((phi_norm - sin(phi_norm)) / phi_norm) * W*W;
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 3, 3> SO3_left_jacobian_inv(const Eigen::Matrix<ScalarType, 3, 1>& phi)
{
    constexpr double kTolerance = 1e-10;
    const ScalarType phi_norm = phi.norm();
    if (phi_norm < kTolerance) {
        return Eigen::Matrix<ScalarType, 3, 3>::Identity() - 0.5 * SO3_hat<ScalarType>(phi);
    }
    const Eigen::Matrix<ScalarType, 3, 3> W = SO3_hat<ScalarType>(phi) / phi_norm;
    const ScalarType phi_norm_half = phi_norm / 2.0;
    return Eigen::Matrix<ScalarType, 3, 3>::Identity() - (phi_norm_half) * W + (1.0 - ((phi_norm_half) / tan(phi_norm_half))) * W*W;
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 6, 1> SE2_log(const Pose<ScalarType>& pose)
{
    const auto phi = SO3_log(pose.rotation);
    const auto pos = pose.position;

    // constexpr double kTolerance = 1e-10;
    // const ScalarType phi_norm = phi.norm();
    // // TODO: Pre-allocate result variable since it is used in both branches?
    // if (phi_norm < kTolerance)
    // {
    //     Eigen::Matrix<ScalarType, 6, 1> result;
    //     result << phi, pos;
    //     return result;
    // }

    const Eigen::Matrix<ScalarType, 3, 3> Jinv = SO3_left_jacobian_inv(phi);
    Eigen::Matrix<ScalarType, 6, 1> result;
    result << phi, Jinv * pos;
    return result;
}

template<typename ScalarType>
Pose<ScalarType> SE2_exp(const Eigen::Matrix<ScalarType, 6, 1>& tau)
{
    const Eigen::Matrix<ScalarType, 3, 1>& phi = tau.template segment<3>(0);
    const Eigen::Matrix<ScalarType, 3, 1>& rho = tau.template segment<3>(3);

    // constexpr double kTolerance = 1e-10;
    // const ScalarType phi_norm = phi.norm();
    // if (phi_norm < kTolerance) {
    //     return Pose<ScalarType>{Eigen::Matrix<ScalarType, 3, 3>::Identity(), rho};
    // }

    const Eigen::Matrix<ScalarType, 3, 3> J = SO3_left_jacobian(phi);
    return Pose<ScalarType>{SO3_exp(phi), J * rho};
}

template<typename ScalarType>
Pose<ScalarType> oplus(const Pose<ScalarType>& lhs, const Eigen::Matrix<ScalarType, 6, 1>& rhs)
{
    return lhs * SE2_exp(rhs);
}

template<typename ScalarType>
Eigen::Matrix<ScalarType, 6, 1> ominus(const Pose<ScalarType>& lhs, const Pose<ScalarType>& rhs)
{
    return SE2_log(inverse(rhs) * lhs);
}

} // namespace pet::mpc

#endif // PET_CONTROL_PARAMETERIZATION_H
