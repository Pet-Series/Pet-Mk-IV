#include "kalman_filter.h"

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
    const auto pos   = position();
    const auto vel   = velocity();

    const ugl::Vector<2> acc2d{acc.x(), acc.y()};
    const ugl::lie::Rotation2D R{theta};

    // State propagation
    const double new_theta = theta + ang_vel.z() * dt;
    const ugl::Vector<2> new_pos = pos + (R * vel * dt) + (R * acc2d * 0.5*dt*dt);
    const ugl::Vector<2> new_vel = vel + acc2d * dt;

    // Error propagation
    // TODO: Calculate jacobians A and B.
    Jacobian<5,5> A; // df/d{state}
    Jacobian<5,3> B; // df/d{noise}

    // TODO: Estimate real noise values.
    Covariance<3> Q_imu = Covariance<3>::Identity() * 0.1;

    m_P = A*m_P*A.transpose() + B*Q_imu*B.transpose();

    set_heading(new_theta);
    set_position(new_pos);
    set_velocity(new_vel);
}

void KalmanFilter::velocity_update(double velocity)
{
    // TODO: Calculate jacobians H and M.
    Jacobian<1,5> H; // dh/d{state}
    Jacobian<1,1> M; // dh/d{noise}

    // TODO: Estimate real noise values.
    Covariance<1> Q_vel = Covariance<1>::Identity() * 0.1;

    const Covariance<1> S = H*m_P*H.transpose() + M*Q_vel*M.transpose();
    const ugl::Matrix<5,1> K = m_P*H.transpose()*S.inverse();

    const double innovation = velocity - m_X[kIndexVelX];

    m_X = m_X + K*innovation;

    m_P = (Covariance<5>::Identity() - K*H) * m_P;
}

} // namespace pet
