#ifndef PET_CONTROL_MPC_H
#define PET_CONTROL_MPC_H

#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <ceres/ceres.h>

#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

class Mpc
{
public:
    struct Options
    {
        double time_horizon = 2.0;
        double time_step = 0.01;

        int max_penalty_iterations = 8;
        double penalty_increase_factor = 5.0;
        double max_constraint_cost = 10e-3;

        double reference_loss_factor = 20.0;
        double velocity_loss_factor = 1.0;
    };

    struct Setpoint
    {
        double time_stamp;
        Pose2D<double> pose;

        Setpoint() = default;
        Setpoint(double t_time_stamp, Pose2D<double> t_pose);
    };

public:
    Mpc(const KinematicModel& kinematic_model, const Options& options);

    void set_reference_path(const nav_msgs::Path& reference_path_ros);
    void set_reference_path(const std::vector<Setpoint>& reference_path);

    void set_initial_pose(const geometry_msgs::PoseStamped& initial_pose);
    void set_initial_pose(const Pose2D<double>& initial_pose);

    void set_initial_twist(const geometry_msgs::TwistStamped& initial_twist);
    void set_initial_twist(const Pose2D<double>::TangentType& initial_twist);

    std::vector<Setpoint> get_optimal_path() const;

    void solve();

private:
    void build_optimization_problem(ceres::Problem& problem);

    /// @brief Generate initial values from initial pose and twist assuming no change in twist over time.
    void generate_initial_values();

    bool is_kinematically_feasible(const ceres::Problem& problem) const;

private:
    KinematicModel m_kinematic_model;
    Options m_options{};

    Pose2D<double> m_initial_pose = Pose2D<double>::Identity();
    Pose2D<double>::TangentType m_initial_twist = Pose2D<double>::TangentType::Zero();

    std::vector<Pose2D<double>> m_optimal_path{};
    std::vector<Pose2D<double>> m_reference_path{};
    std::vector<Pose2D<double>::TangentType> m_twists{};

    bool m_reference_path_is_set = false;
    int m_problem_size = 0;
    ceres::ScaledLoss m_reference_loss_function;
    ceres::ScaledLoss m_velocity_loss_function;
    ceres::LossFunctionWrapper m_constraint_penalty_coefficient_handle;

    std::vector<ceres::ResidualBlockId> m_kinematic_constraint_residuals{};
};

} // namespace pet::control

#endif // PET_CONTROL_MPC_H
