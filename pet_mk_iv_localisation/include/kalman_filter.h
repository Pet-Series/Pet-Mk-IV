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

    double heading() const { return m_X[kIndexTheta]; }
    ugl::Vector<2> position() const { return m_X.segment<2>(kIndexPosX); }
    ugl::Vector<2> velocity() const { return m_X.segment<2>(kIndexVelX); }

    void set_heading(double theta) { m_X[kIndexTheta] = theta; }
    void set_position(const ugl::Vector<2>& position) { m_X.segment<2>(kIndexPosX) = position; }
    void set_velocity(const ugl::Vector<2>& velocity) { m_X.segment<2>(kIndexVelX) = velocity; }

    // Predicts new state from time passed and accelerometer+gyroscope measurements.
    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

    // Updates state estimation from a measurement of forward velocity in body frame.
    void velocity_update(double velocity);

private:
    // State vector {theta, pos, vel}.
    ugl::Vector<5> m_X = ugl::Vector<5>::Zero();

    // Error covariance {theta, pos, vel}.
    Covariance<5> m_P = Covariance<5>::Identity() * 0.1;

    static constexpr int kIndexTheta = 0;
    static constexpr int kIndexPosX = 1;
    static constexpr int kIndexPosY = 2;
    static constexpr int kIndexVelX = 3;
    static constexpr int kIndexVelY = 4;
};

} // namespace pet

#endif // PET_LOCALISATION_KALMAN_FILTER_H