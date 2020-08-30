#include "kalman_filter.h"

#include <cmath>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation2d.h>

namespace pet
{

KalmanFilter::KalmanFilter(double theta, const ugl::Vector<2>& position, const ugl::Vector<2>& velocity)
{
    set_heading(theta);
    set_position(position);
    set_velocity(velocity);
}

void KalmanFilter::predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel)
{
    const auto theta = heading();
    const auto vel   = velocity();
    const auto pos   = position();

    const ugl::Vector<2> acc2d{acc.x(), acc.y()};
    const ugl::lie::Rotation2D R{theta};

    // State propagation
    const double new_theta = theta + ang_vel.z() * dt;
    const ugl::Vector<2> new_vel = vel + acc2d * dt;
    const ugl::Vector<2> new_pos = pos + (R * vel * dt) + (R * acc2d * 0.5*dt*dt);

    // Error propagation
    const Jacobian<5,5> A = prediction_state_jacobian(dt, m_X, acc2d);
    const Jacobian<5,3> B = prediction_noise_jacobian(dt, m_X);

    // TODO: Estimate real noise values.
    Covariance<3> Q_imu = Covariance<3>::Identity() * 0.1;

    m_P = A*m_P*A.transpose() + B*Q_imu*B.transpose();

    set_heading(new_theta);
    set_position(new_pos);
    set_velocity(new_vel);
}

void KalmanFilter::sonar_velocity_update(double velocity)
{
    Jacobian<1,5> H = Jacobian<1,5>::Zero();
    H[kIndexVelX] = 1.0;
    Jacobian<1,1> G = Jacobian<1,1>::Identity();

    // TODO: Estimate real noise values.
    Covariance<1> Q_vel = Covariance<1>::Identity() * 0.1;

    const Covariance<1> S = H*m_P*H.transpose() + G*Q_vel*G.transpose();
    const ugl::Matrix<5,1> K = m_P*H.transpose()*S.inverse();

    const double innovation = velocity - m_X[kIndexVelX];

    m_X = m_X + K*innovation;

    m_P = (Covariance<5>::Identity() - K*H) * m_P;
}

void KalmanFilter::pseudo_lateral_velocity_update(double velocity)
{
    Jacobian<1,5> H = Jacobian<1,5>::Zero();
    H[kIndexVelY] = 1.0;
    Jacobian<1,1> G = Jacobian<1,1>::Identity();

    // TODO: How certain certain should we claim to be?
    Covariance<1> Q_vel = Covariance<1>::Identity() * 0.01;

    const Covariance<1> S = H*m_P*H.transpose() + G*Q_vel*G.transpose();
    const ugl::Matrix<5,1> K = m_P*H.transpose()*S.inverse();

    const double innovation = velocity - m_X[kIndexVelY];

    m_X = m_X + K*innovation;

    m_P = (Covariance<5>::Identity() - K*H) * m_P;
}

KalmanFilter::Jacobian<5,5> KalmanFilter::prediction_state_jacobian(double dt, const ugl::Vector<5> X, const ugl::Vector<2>& acc)
{
    const double theta = X[kIndexTheta];
    const ugl::Vector<2> vel = X.segment<2>(kIndexVelX);
    const ugl::lie::Rotation2D R{theta};

    // Derivative of R with regards to theta.
    ugl::Matrix<2,2> dRdtheta;
    dRdtheta << -std::sin(theta), -std::cos(theta),
                 std::cos(theta), -std::sin(theta);

    Jacobian<5,5> A = Jacobian<5,5>::Identity();
    A.block<2,1>(kIndexPosX, kIndexTheta) = dRdtheta * (vel*dt + 0.5*acc*dt*dt);
    A.block<2,2>(kIndexPosX, kIndexVelX)  = R.matrix() * dt;
    return A;
}

KalmanFilter::Jacobian<5,3> KalmanFilter::prediction_noise_jacobian(double dt, const ugl::Vector<5> X)
{
    const ugl::lie::Rotation2D R{X[kIndexTheta]};
    Jacobian<5,3> B = Jacobian<5,3>::Identity() * dt;
    B.block<2,2>(kIndexPosX, 1) = R.matrix() * 0.5*dt*dt;
    return B;
}

} // namespace pet
