#include "pet_mk_iv_control/mpc_node.h"

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include <ugl/math/quaternion.h>

#include <ugl_ros/convert_tf2.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

Mpc::Options load_mpc_parameters(ros::NodeHandle& nh)
{
    Mpc::Options options{};
    options.max_num_poses           = nh.param("mpc/max_num_poses", options.max_num_poses);
    options.time_step               = nh.param("mpc/time_step", options.time_step);
    options.max_penalty_iterations  = nh.param("mpc/max_penalty_iterations", options.max_penalty_iterations);
    options.penalty_increase_factor = nh.param("mpc/penalty_increase_factor", options.penalty_increase_factor);
    options.max_constraint_cost     = nh.param("mpc/max_constraint_cost", options.max_constraint_cost);
    options.reference_loss_factor   = nh.param("mpc/reference_loss_factor", options.reference_loss_factor);
    options.velocity_loss_factor    = nh.param("mpc/velocity_loss_factor", options.velocity_loss_factor);
    return options;
}

KinematicModel::Parameters load_kinematic_parameters(ros::NodeHandle& nh)
{
    KinematicModel::Parameters params{};
    params.max_linear_speed         = nh.param("kinematics/max_linear_speed", params.max_linear_speed);
    params.max_angular_speed        = nh.param("kinematics/max_angular_speed", params.max_angular_speed);
    return params;
}

Mpc create_mpc_solver()
{
    ros::NodeHandle nh_private("~");
    auto model = KinematicModel{load_kinematic_parameters(nh_private)};
    auto options = load_mpc_parameters(nh_private);
    return Mpc{model, options};
}

std::vector<Pose2D<double>> create_reference_path()
{
    const Pose2D<double> initial_pose = Pose2D<double>::Identity();

    const double yaw_vel = 0.0;
    const double x_vel   = 0.3;
    const double y_vel   = 0.0;
    const Pose2D<double>::TangentType twist{yaw_vel, x_vel, y_vel};

    std::vector<Pose2D<double>> reference_path;
    reference_path.push_back(initial_pose);
    for (int i = 0; i < 100; ++i)
    {
        constexpr double dt = 0.05;
        const auto pose = KinematicModel::propagate(reference_path.back(), twist, dt);
        reference_path.push_back(pose);
    }

    return reference_path;
}

nav_msgs::Path to_path_msg(const std::vector<Pose2D<double>>& path)
{
    nav_msgs::Path path_msg{};
    path_msg.header.frame_id = "map";
    for (const auto& pose: path)
    {
        geometry_msgs::PoseStamped pose_msg{};
        pose_msg.pose.orientation = tf2::toMsg(SO2<double>::to_quaternion(pose.rotation));
        pose_msg.pose.position.x = pose.position.x();
        pose_msg.pose.position.y = pose.position.y();
        path_msg.poses.push_back(pose_msg);
    }
    return path_msg;
}

} // namespace pet::control


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    google::InitGoogleLogging(argv[0]);

    auto initial_pose = pet::control::Pose2D<double>::Identity();
    const double initial_yaw = 90.0 * 3.1415/180.0;
    initial_pose.rotation = pet::control::SO2<double>::exp(initial_yaw);

    const double yaw_vel = 0.0;
    const double x_vel   = 0.4;
    const double y_vel   = 0.0;
    const pet::control::Pose2D<double>::TangentType initial_twist{yaw_vel, x_vel, y_vel};

    const auto reference_path = pet::control::create_reference_path();

    auto solver = pet::control::create_mpc_solver();
    solver.set_reference_path(reference_path);
    solver.set_initial_pose(initial_pose);
    solver.set_initial_twist(initial_twist);
    solver.solve();
    const auto optimal_path = solver.get_optimal_path();

    ros::NodeHandle nh("");
    ros::Publisher reference_path_publisher = nh.advertise<nav_msgs::Path>("reference_path", 10, true);
    ros::Publisher optimal_path_publisher = nh.advertise<nav_msgs::Path>("optimal_path", 10, true);

    reference_path_publisher.publish(pet::control::to_path_msg(reference_path));
    optimal_path_publisher.publish(pet::control::to_path_msg(optimal_path));

    ros::spin();

    return 0;
}
