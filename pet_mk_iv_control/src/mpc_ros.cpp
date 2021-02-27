#include "pet_mk_iv_control/mpc_ros.h"

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <ugl_ros/convert_tf2.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"


namespace pet::control
{

Mpc::Options load_mpc_parameters(ros::NodeHandle& nh)
{
    Mpc::Options options{};
    options.time_horizon           = nh.param("mpc/time_horizon", options.time_horizon);
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

nav_msgs::Path to_path_msg(const std::vector<Mpc::Setpoint>& path)
{
    nav_msgs::Path path_msg{};
    path_msg.header.frame_id = "map";
    for (const auto& setpoint: path)
    {
        geometry_msgs::PoseStamped pose_msg{};
        pose_msg.header.stamp = ros::Time(setpoint.time_stamp);
        pose_msg.pose.orientation = tf2::toMsg(SO2<double>::to_quaternion(setpoint.pose.rotation));
        pose_msg.pose.position.x = setpoint.pose.position.x();
        pose_msg.pose.position.y = setpoint.pose.position.y();
        path_msg.poses.push_back(pose_msg);
    }
    return path_msg;
}

} // namespace pet::control
