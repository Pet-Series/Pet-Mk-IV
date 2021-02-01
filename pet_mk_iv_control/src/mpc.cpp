#include "pet_mk_iv_control/mpc.h"

#include <algorithm>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <ceres/ceres.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <ugl/math/quaternion.h>
#include <ugl/lie_group/pose.h>

#include <ugl_ros/convert_tf2.h>
#include <ugl_ros/convert_ugl.h>

#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/parameterization2d.h"
#include "pet_mk_iv_control/residuals.h"

namespace pet::control
{

Mpc::Mpc(const Options& options)
    : m_options(options)
    , m_rotations(1, Eigen::Matrix2d::Identity())
    , m_positions(1, Eigen::Vector2d::Zero())
    , m_twists(1, Eigen::Vector3d::Zero())
    , m_reference_loss_function(nullptr, m_options.reference_loss_factor, ceres::TAKE_OWNERSHIP)
    , m_velocity_loss_function(nullptr, m_options.velocity_loss_factor, ceres::TAKE_OWNERSHIP)
    , m_constraint_penalty_coefficient_handle(nullptr, ceres::TAKE_OWNERSHIP)
{
}

void Mpc::set_reference_path(const nav_msgs::Path& reference_path)
{
    ROS_ASSERT(reference_path.poses.size() > 0);
    m_problem_size = std::min(reference_path.poses.size(), m_options.max_num_poses);

    m_reference_positions.clear();
    m_reference_rotations.clear();
    m_reference_positions.reserve(m_problem_size);
    m_reference_rotations.reserve(m_problem_size);

    /// TODO: Transform reference_path into correct tf frame.
    for (std::size_t i = 0; i < m_problem_size; ++i)
    {
        const geometry_msgs::Pose& pose = reference_path.poses[i].pose;
        m_reference_positions.emplace_back(pose.position.x, pose.position.y);
        m_reference_rotations.emplace_back(SO2<double>::from_quaternion(tf2::fromMsg(pose.orientation)));
    }
    m_reference_path_set = true;
}

void Mpc::set_initial_pose(const geometry_msgs::PoseStamped& initial_pose)
{
    m_positions.clear();
    m_rotations.clear();
    /// TODO: Transform initial_pose into correct tf frame.
    const geometry_msgs::Pose& pose = initial_pose.pose;
    m_positions.emplace_back(pose.position.x, pose.position.y);
    m_rotations.emplace_back(SO2<double>::from_quaternion(tf2::fromMsg(pose.orientation)));
}

void Mpc::set_initial_twist(const geometry_msgs::TwistStamped& initial_twist)
{
    m_twists.clear();
    /// TODO: Transform initial_twist into correct tf frame.
    const auto& twist = initial_twist.twist;
    m_twists.emplace_back(twist.angular.z, twist.linear.x, twist.linear.y);
}

void Mpc::solve()
{
    if (!m_reference_path_set) {
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
        m_constraint_penalty_coefficient_handle.Reset(new ceres::ScaledLoss{nullptr, penalty_coefficient, ceres::TAKE_OWNERSHIP}, ceres::TAKE_OWNERSHIP);
        ceres::Solve(solver_options, &problem, &summary);
        std::cout << summary.FullReport() << "\n";

        /// TODO: If constraint_residual[i] < kMaxConstraintCost then break here.

        ++iteration;
        penalty_coefficient *= m_options.penalty_increase_factor;
    }
    if (iteration >= m_options.max_penalty_iterations) {
        ROS_WARN("Max constraint penalty iterations reached.");
    }

    // std::cout << summary.BriefReport() << "\n";
    // std::cout << summary.FullReport() << "\n";
}

nav_msgs::Path Mpc::get_optimal_path() const
{
    nav_msgs::Path optimal_path{};
    optimal_path.header.frame_id = "map";
    for (std::size_t i = 0; i < m_problem_size; ++i)
    {
        geometry_msgs::PoseStamped pose{};
        pose.pose.orientation = tf2::toMsg(SO2<double>::to_quaternion(m_rotations[i]));
        pose.pose.position.x = m_positions[i].x();
        pose.pose.position.y = m_positions[i].y();
        optimal_path.poses.push_back(pose);
    }
    return optimal_path;
}

void Mpc::build_optimization_problem(ceres::Problem& problem)
{
    // ceres::LossFunction* no_loss_function = nullptr;
    ceres::LocalParameterization* rotation2d_parameterization = new Rotation2DParameterization{};
    ceres::LocalParameterization* twist_diffdrive_parameterization = new ceres::SubsetParameterization{3, {2}};

    // Initial pose & twist are constant parameters.
    problem.AddParameterBlock(m_rotations[0].data(), 4, rotation2d_parameterization);
    problem.AddParameterBlock(m_positions[0].data(), 2);
    problem.AddParameterBlock(m_twists[0].data(), 3, twist_diffdrive_parameterization);
    problem.SetParameterBlockConstant(m_rotations[0].data());
    problem.SetParameterBlockConstant(m_positions[0].data());
    problem.SetParameterBlockConstant(m_twists[0].data());

    // Start loop with the second element. For the first element the reference path
    // residual is constant and the velocity residual is undefined.
    for (std::size_t i = 1; i < m_problem_size; ++i)
    {
        problem.AddParameterBlock(m_rotations[i].data(), 4, rotation2d_parameterization);
        problem.AddParameterBlock(m_positions[i].data(), 2);

        problem.AddParameterBlock(m_reference_rotations[i].data(), 4, rotation2d_parameterization);
        problem.AddParameterBlock(m_reference_positions[i].data(), 2);

        problem.AddParameterBlock(m_twists[i].data(), 3, twist_diffdrive_parameterization);

        // Do not optimize over reference path parameters.
        problem.SetParameterBlockConstant(m_reference_rotations[i].data());
        problem.SetParameterBlockConstant(m_reference_positions[i].data());

        // Residual block for reference path error.
        problem.AddResidualBlock(
                ReferencePathResidual::Create(),
                &m_reference_loss_function,
                m_reference_rotations[i].data(),
                m_reference_positions[i].data(),
                m_rotations[i].data(),
                m_positions[i].data()
        );

        // Residual block for change in velocity.
        problem.AddResidualBlock(
                VelocityChangeResidual::Create(),
                &m_velocity_loss_function,
                m_twists[i].data(),
                m_twists[i-1].data()
        );

        // Residual block for kinematic constraint penalty.
        problem.AddResidualBlock(
                KinematicConstraintPenaltyResidual::Create(m_options.time_step),
                &m_constraint_penalty_coefficient_handle,
                m_rotations[i].data(),
                m_positions[i].data(),
                m_rotations[i-1].data(),
                m_positions[i-1].data(),
                m_twists[i-1].data()
        );
    }
}

void Mpc::generate_initial_values()
{
    const auto twist = m_twists[0];
    const auto dt = m_options.time_step;
    auto prev_pose = Pose2D<double>{m_rotations[0], m_positions[0]};
    for (std::size_t i = 1; i < m_problem_size; ++i)
    {
        const auto& next_pose = KinematicModel::propagate(prev_pose, twist, dt);
        m_positions.push_back(next_pose.position);
        m_rotations.push_back(next_pose.rotation);
        m_twists.push_back(twist);
        prev_pose = next_pose;
    }
}

} // namespace pet::control


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    google::InitGoogleLogging(argv[0]);

    nav_msgs::Path reference_path;
    reference_path.header.frame_id = "map";
    for (int i = 0; i < 50; ++i)
    {
        geometry_msgs::PoseStamped pose{};
        pose.pose.position.x = 0.1 * i;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        reference_path.poses.push_back(pose);
    }

    geometry_msgs::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.pose.position.x = 0.0;
    initial_pose.pose.position.y = 0.0;
    initial_pose.pose.position.z = 0.0;
    const auto initial_quat = ugl::math::to_quat(90.0*3.1415/180.0, Eigen::Vector3d::UnitZ());
    initial_pose.pose.orientation = tf2::toMsg(initial_quat);

    /// TODO: Read options from ROS-parameters.
    pet::mpc::Mpc solver{pet::mpc::Options{}};
    solver.set_reference_path(reference_path);
    solver.set_initial_pose(initial_pose);
    solver.solve();
    const nav_msgs::Path optimal_path = solver.get_optimal_path();

    ros::NodeHandle nh("");
    ros::Publisher reference_path_publisher = nh.advertise<nav_msgs::Path>("reference_path", 10, true);
    ros::Publisher optimal_path_publisher = nh.advertise<nav_msgs::Path>("optimal_path", 10, true);

    reference_path_publisher.publish(reference_path);
    optimal_path_publisher.publish(optimal_path);

    ros::spin();

    return 0;
}
