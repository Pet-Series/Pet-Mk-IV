#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <ugl/math/quaternion.h>

#include <ugl_ros/convert_tf2.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/mpc_ros.h"
#include "pet_mk_iv_control/kinematic_model.h"


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

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    auto model = pet::control::KinematicModel{pet::control::load_kinematic_parameters(nh_private)};
    auto options = pet::control::load_mpc_parameters(nh_private);
    pet::control::Mpc solver{model, options};
    solver.set_reference_path(reference_path);
    solver.set_initial_pose(initial_pose);
    solver.solve();
    const auto optimal_path = solver.get_optimal_path();

    ros::Publisher reference_path_publisher = nh.advertise<nav_msgs::Path>("reference_path", 10, true);
    ros::Publisher optimal_path_publisher = nh.advertise<nav_msgs::Path>("optimal_path", 10, true);

    reference_path_publisher.publish(reference_path);
    optimal_path_publisher.publish(pet::control::to_path_msg(optimal_path));

    ros::spin();

    return 0;
}
