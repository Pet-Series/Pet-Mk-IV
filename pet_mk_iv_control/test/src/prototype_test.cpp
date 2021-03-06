#include "pet_mk_iv_control/mpc_node.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/mpc_ros.h"
#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"
#include "pet_mk_iv_control/setpoint.h"
#include "pet_mk_iv_control/trajectory.h"

namespace pet::control
{

Mpc create_mpc_solver()
{
    ros::NodeHandle nh_private("~");
    auto model = KinematicModel{load_kinematic_parameters(nh_private)};
    auto options = load_mpc_parameters(nh_private);
    return Mpc{model, options};
}

LinearTrajectory create_reference_path()
{
    const Pose2D<double> initial_pose = Pose2D<double>::Identity();

    const double yaw_vel = 0.0;
    const double x_vel   = 0.3;
    const double y_vel   = 0.0;
    const Pose2D<double>::TangentType twist{yaw_vel, x_vel, y_vel};

    std::vector<Setpoint> reference_path;
    reference_path.emplace_back(0.0, initial_pose);
    for (int i = 1; i <= 100; ++i)
    {
        constexpr double dt = 0.05;
        const auto pose = KinematicModel::propagate(reference_path.back().pose, twist, dt);
        reference_path.emplace_back(dt*i, pose);
    }

    return LinearTrajectory{reference_path};
}

} // namespace pet::control


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh{""};

    google::InitGoogleLogging(argv[0]);

    auto initial_pose = pet::control::Pose2D<double>::Identity();
    const double initial_yaw = 90.0 * 3.1415/180.0;
    initial_pose.rotation = pet::control::SO2<double>::exp(initial_yaw);

    const double yaw_vel = 0.0;
    const double x_vel   = 0.4;
    const double y_vel   = 0.0;
    const auto initial_twist = pet::control::Pose2D<double>::TangentType{yaw_vel, x_vel, y_vel};

    const auto reference_path = pet::control::create_reference_path();

    auto solver = pet::control::create_mpc_solver();
    solver.set_reference_path(reference_path);
    solver.set_initial_pose(initial_pose);
    solver.set_initial_twist(initial_twist);
    solver.solve();
    const auto optimal_path = solver.get_optimal_path();

    ros::Publisher reference_path_publisher = nh.advertise<nav_msgs::Path>("reference_path", 10, true);
    ros::Publisher optimal_path_publisher = nh.advertise<nav_msgs::Path>("optimal_path", 10, true);

    reference_path_publisher.publish(pet::control::to_path_msg(reference_path.get_control_points()));
    optimal_path_publisher.publish(pet::control::to_path_msg(optimal_path));

    ros::spin();

    return 0;
}
