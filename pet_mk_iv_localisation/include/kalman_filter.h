#ifndef PET_LOCALISATION_KALMAN_FILTER_H
#define PET_LOCALISATION_KALMAN_FILTER_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace pet
{

class KalmanFilter
{
public:
    template<int n>
    using Covariance = ugl::Matrix<n, n>;

    template<int rows, int cols>
    using Jacobian = ugl::Matrix<rows, cols>;

public:
    KalmanFilter() = default;
    KalmanFilter(double theta, const ugl::Vector<2>& position, const ugl::Vector<2>& velocity);

    double heading() const { return m_theta; }
    const ugl::Vector<2>& position() const { return m_pos; }
    const ugl::Vector<2>& velocity() const { return m_vel; }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

private:
    // Estimated heading in reference frame. [rad]
    double m_theta = 0.0;

    // Estimated position in reference frame. [m]
    ugl::Vector<2> m_pos = ugl::Vector<2>::Zero();

    // Estimated velocity in body frame. [m/s]
    ugl::Vector<2> m_vel = ugl::Vector<2>::Zero();

    // Uncertainty of estimated state {theta, pos, vel}.
    Covariance<5> m_P = Covariance<5>::Identity() * 0.1;
};

} // namespace pet

#endif // PET_LOCALISATION_KALMAN_FILTER_H