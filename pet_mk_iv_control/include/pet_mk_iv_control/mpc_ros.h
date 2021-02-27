#ifndef PET_CONTROL_MPC_ROS_H
#define PET_CONTROL_MPC_ROS_H

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/kinematic_model.h"
#include "pet_mk_iv_control/pose2d.h"

namespace pet::control
{

Mpc::Options load_mpc_parameters(ros::NodeHandle& nh);

KinematicModel::Parameters load_kinematic_parameters(ros::NodeHandle& nh);

nav_msgs::Path to_path_msg(const std::vector<Mpc::Setpoint>& path);

} // namespace pet::control

#endif // PET_CONTROL_MPC_ROS_H
