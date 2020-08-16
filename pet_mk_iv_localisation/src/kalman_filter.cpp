#include "kalman_filter.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation2d.h>

namespace pet
{

KalmanFilter::KalmanFilter(double theta, const ugl::Vector<2>& position, const ugl::Vector<2>& velocity)
    : m_theta(theta)
    , m_pos(position)
    , m_vel(velocity)
{
}

void KalmanFilter::predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel)
{
    const ugl::Vector<2> acc2d{acc.x(), acc.y()};
    const ugl::lie::Rotation2D R{m_theta};

    // State propagation
    const double new_theta = m_theta + ang_vel.z() * dt;
    const ugl::Vector<2> new_pos = m_pos + (R * m_vel * dt) + (R * acc2d * 0.5*dt*dt);
    const ugl::Vector<2> new_vel = m_vel + acc2d * dt;

    // Error propagation
    Jacobian<5,5> A = Jacobian<5,5>::Zero(); // df/dX
    Jacobian<5,3> B = Jacobian<5,3>::Zero(); // df/dv

    m_theta = new_theta;
    m_pos = new_pos;
    m_vel = new_vel;
}

} // namespace pet
