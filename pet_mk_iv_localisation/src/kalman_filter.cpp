#include "kalman_filter.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

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
    // TODO: Implement this!
}

} // namespace pet
