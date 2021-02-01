#ifndef PET_CONTROL_MPC_H
#define PET_CONTROL_MPC_H

#include <array>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <ceres/ceres.h>

#include "pet_mk_iv_control/parameterization2d.h"

namespace pet::control
{

struct Options
{
    std::size_t max_num_poses = 100;

    double time_step = 0.01;

    int max_penalty_iterations = 8;
    double penalty_increase_factor = 5.0;
    double max_constraint_cost = 10e-3;

    double reference_loss_factor = 20.0;
    double velocity_loss_factor = 1.0;
};

class Mpc
{
public:
    Mpc(const Options& options);

    void set_reference_path(const nav_msgs::Path& reference_path);

    void set_initial_pose(const geometry_msgs::PoseStamped& initial_pose);

    void set_initial_twist(const geometry_msgs::TwistStamped& initial_twist);

    void solve();

    nav_msgs::Path get_optimal_path() const;

private:
    void build_optimization_problem(ceres::Problem& problem);

    /// @brief Generate initial values from initial pose and twist assuming no change in twist over time.
    void generate_initial_values();

private:
    Options m_options{};

    std::vector<Pose2D<double>::RotationType> m_rotations{};
    std::vector<Pose2D<double>::PointType> m_positions{};

    std::vector<Pose2D<double>::RotationType> m_reference_rotations{};
    std::vector<Pose2D<double>::PointType> m_reference_positions{};

    std::vector<Pose2D<double>::TangentType> m_twists{};

    bool m_reference_path_set = false;
    std::size_t m_problem_size = 0;
    ceres::ScaledLoss m_reference_loss_function;
    ceres::ScaledLoss m_velocity_loss_function;
    ceres::LossFunctionWrapper m_constraint_penalty_coefficient_handle;
};

} // namespace pet::control

#endif // PET_CONTROL_MPC_H
