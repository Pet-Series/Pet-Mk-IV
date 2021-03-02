#include "pet_mk_iv_control/mpc.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <ceres/ceres.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <ugl_ros/convert_tf2.h>
#include <ugl_ros/convert_ugl.h>

#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"
#include "pet_mk_iv_control/residuals.h"

namespace pet::control
{

Mpc::Setpoint::Setpoint(double t_time_stamp, Pose2D<double> t_pose)
    : time_stamp(t_time_stamp)
    , pose(t_pose)
{}

Mpc::Mpc(const KinematicModel& kinematic_model, const Options& options)
    : m_kinematic_model(kinematic_model)
    , m_options(options)
    , m_reference_loss_function(nullptr, m_options.reference_loss_factor, ceres::TAKE_OWNERSHIP)
    , m_velocity_loss_function(nullptr, m_options.velocity_loss_factor, ceres::TAKE_OWNERSHIP)
    , m_constraint_penalty_coefficient_handle(nullptr, ceres::TAKE_OWNERSHIP)
{
    m_problem_size = std::ceil(m_options.time_horizon / m_options.time_step);
}

void Mpc::set_reference_path(const nav_msgs::Path& reference_path_ros)
{
    ROS_ASSERT_MSG(!reference_path_ros.poses.empty(), "Provided reference path must be non-empty!");
    /// TODO: Transform reference_path into correct tf frame.
    std::vector<Setpoint> reference_path{};
    reference_path.reserve(reference_path_ros.poses.size());
    const auto t0 = reference_path_ros.poses.front().header.stamp;
    std::transform(
        std::cbegin(reference_path_ros.poses), std::cend(reference_path_ros.poses),
        std::back_insert_iterator(reference_path),
        [&](const geometry_msgs::PoseStamped& in_pose) {
            Setpoint setpoint;
            setpoint.time_stamp = (in_pose.header.stamp - t0).toSec();
            setpoint.pose.position.x() = in_pose.pose.position.x;
            setpoint.pose.position.y() = in_pose.pose.position.y;
            setpoint.pose.rotation = SO2<double>::from_quaternion(tf2::fromMsg(in_pose.pose.orientation));
            return setpoint;
        }
    );
    set_reference_path(reference_path);
}

void Mpc::set_reference_path(const std::vector<Setpoint>& reference_path)
{
    ROS_ASSERT_MSG(!reference_path.empty(), "Provided reference path must be non-empty!");

    /// TODO: Interpolate between poses in reference_path so that m_reference_path have desired timestep and size.
    m_reference_path.clear();
    m_reference_path.reserve(m_problem_size);
    std::transform(
        std::cbegin(reference_path), std::cend(reference_path),
        std::back_insert_iterator(m_reference_path),
        [](const Setpoint& setpoint) {
            return setpoint.pose;
        }
    );
    m_reference_path_is_set = true;
}

void Mpc::set_initial_pose(const geometry_msgs::PoseStamped& initial_pose_ros)
{
    /// TODO: Transform initial_pose into correct tf frame.
    Pose2D<double> initial_pose{};
    initial_pose.position.x() = initial_pose_ros.pose.position.x;
    initial_pose.position.y() = initial_pose_ros.pose.position.y;
    initial_pose.rotation = SO2<double>::from_quaternion(tf2::fromMsg(initial_pose_ros.pose.orientation));
    set_initial_pose(initial_pose);
}

void Mpc::set_initial_pose(const Pose2D<double>& initial_pose)
{
    m_initial_pose = initial_pose;
}

void Mpc::set_initial_twist(const geometry_msgs::TwistStamped& initial_twist_ros)
{
    /// TODO: Transform initial_twist into correct tf frame.
    const auto& twist = initial_twist_ros.twist;
    set_initial_twist(Pose2D<double>::TangentType{twist.angular.z, twist.linear.x, twist.linear.y});
}

void Mpc::set_initial_twist(const Pose2D<double>::TangentType& initial_twist)
{
    m_initial_twist = initial_twist;
}

std::vector<Mpc::Setpoint> Mpc::get_optimal_path() const
{
    /// TODO: Assert solution is found.
    std::vector<Setpoint> setpoints{};
    setpoints.reserve(m_problem_size);
    for (int i = 0; i < m_problem_size; ++i)
    {
        setpoints.emplace_back(m_options.time_step*i, m_optimal_path.at(i));
    }
    return setpoints;
}

void Mpc::solve()
{
    if (!m_reference_path_is_set) {
        constexpr auto error_text = "Reference path must be set before calling Mpc::solve()!";
        ROS_ERROR(error_text);
        throw error_text;
    }

    generate_initial_values();

    ceres::Problem::Options problem_options;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem{problem_options};
    build_optimization_problem(problem);

    ceres::Solver::Options solver_options;
    // solver_options.check_gradients = true;
    ceres::Solver::Summary summary;

    int iteration = 0;
    double penalty_coefficient = 1;
    while (iteration < m_options.max_penalty_iterations)
    {
        ++iteration;
        m_constraint_penalty_coefficient_handle.Reset(new ceres::ScaledLoss{nullptr, penalty_coefficient, ceres::TAKE_OWNERSHIP}, ceres::TAKE_OWNERSHIP);
        ceres::Solve(solver_options, &problem, &summary);
        // std::cout << summary.FullReport() << "\n";
        ROS_INFO("%s", summary.BriefReport().c_str());

        if (is_kinematically_feasible(problem)) {
            ROS_INFO("Feasible solution found on iteration %i.", iteration);
            break;
        }

        penalty_coefficient *= m_options.penalty_increase_factor;
    }
    if (iteration >= m_options.max_penalty_iterations) {
        ROS_WARN("Max constraint penalty iterations reached.");
    }

    // std::cout << summary.BriefReport() << "\n";
    // std::cout << summary.FullReport() << "\n";
}

void Mpc::build_optimization_problem(ceres::Problem& problem)
{
    // ceres::LossFunction* no_loss_function = nullptr;
    ceres::LocalParameterization* rotation2d_parameterization = new Rotation2DParameterization{};
    ceres::LocalParameterization* twist_diffdrive_parameterization = new ceres::SubsetParameterization{3, {2}};

    // Initial pose & twist are constant parameters.
    problem.AddParameterBlock(m_optimal_path[0].rotation.data(), 4, rotation2d_parameterization);
    problem.AddParameterBlock(m_optimal_path[0].position.data(), 2);
    problem.AddParameterBlock(m_twists[0].data(), 3, twist_diffdrive_parameterization);
    problem.SetParameterBlockConstant(m_optimal_path[0].rotation.data());
    problem.SetParameterBlockConstant(m_optimal_path[0].position.data());
    problem.SetParameterBlockConstant(m_twists[0].data());

    m_kinematic_constraint_residuals.clear();

    // Start loop with the second element. For the first element the reference path
    // residual is constant and the velocity residual is undefined.
    for (int i = 1; i < m_problem_size; ++i)
    {
        problem.AddParameterBlock(m_optimal_path[i].rotation.data(), 4, rotation2d_parameterization);
        problem.AddParameterBlock(m_optimal_path[i].position.data(), 2);

        problem.AddParameterBlock(m_reference_path[i].rotation.data(), 4, rotation2d_parameterization);
        problem.AddParameterBlock(m_reference_path[i].position.data(), 2);

        problem.AddParameterBlock(m_twists[i].data(), 3, twist_diffdrive_parameterization);
        problem.SetParameterUpperBound(m_twists[i].data(), 0, m_kinematic_model.get_max_angular_speed());
        problem.SetParameterUpperBound(m_twists[i].data(), 1, m_kinematic_model.get_max_linear_speed());
        problem.SetParameterLowerBound(m_twists[i].data(), 0, -m_kinematic_model.get_max_angular_speed());
        problem.SetParameterLowerBound(m_twists[i].data(), 1, -m_kinematic_model.get_max_linear_speed());

        // Do not optimize over reference path parameters.
        problem.SetParameterBlockConstant(m_reference_path[i].rotation.data());
        problem.SetParameterBlockConstant(m_reference_path[i].position.data());

        // Residual block for reference path error.
        problem.AddResidualBlock(
                ReferencePathResidual::Create(),
                &m_reference_loss_function,
                m_reference_path[i].rotation.data(),
                m_reference_path[i].position.data(),
                m_optimal_path[i].rotation.data(),
                m_optimal_path[i].position.data()
        );

        // Residual block for change in velocity.
        problem.AddResidualBlock(
                VelocityChangeResidual::Create(),
                &m_velocity_loss_function,
                m_twists[i].data(),
                m_twists[i-1].data()
        );

        // Residual block for kinematic constraint penalty.
        auto residual_id = problem.AddResidualBlock(
                KinematicConstraintPenaltyResidual::Create(m_options.time_step),
                &m_constraint_penalty_coefficient_handle,
                m_optimal_path[i].rotation.data(),
                m_optimal_path[i].position.data(),
                m_optimal_path[i-1].rotation.data(),
                m_optimal_path[i-1].position.data(),
                m_twists[i-1].data()
        );
        m_kinematic_constraint_residuals.push_back(residual_id);
    }
}

void Mpc::generate_initial_values()
{
    m_optimal_path.clear();
    m_optimal_path.reserve(m_problem_size);
    m_optimal_path.push_back(m_initial_pose);
    m_twists.clear();
    m_twists.reserve(m_problem_size);
    m_twists.push_back(m_initial_twist);

    const auto dt = m_options.time_step;
    for (int i = 1; i < m_problem_size; ++i)
    {
        const auto& next_pose = KinematicModel::propagate(m_optimal_path[i-1], m_initial_twist, dt);
        m_optimal_path.push_back(next_pose);
        m_twists.push_back(m_initial_twist);
    }
}

bool Mpc::is_kinematically_feasible(const ceres::Problem& problem) const
{
    for (const auto& id: m_kinematic_constraint_residuals)
    {
        double cost;
        [[maybe_unused]] bool success = problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
        ROS_ASSERT_MSG(success, "Could not evaluate feasibility of kinematic constraint residual block!");
        if (cost > m_options.max_constraint_cost) {
            return false;
        }
    }
    return true;
}

} // namespace pet::control
