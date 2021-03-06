#include "pet_mk_iv_control/mpc_node.h"

#include <ros/ros.h>

#include "pet_mk_iv_control/mpc.h"
#include "pet_mk_iv_control/mpc_ros.h"
#include "pet_mk_iv_control/kinematic_model.h"

namespace pet::control
{

MpcNode::MpcNode()
    : m_nh("")
    , m_nh_private("~")
    , m_mpc(pet::control::KinematicModel{load_kinematic_parameters(m_nh_private)}, load_mpc_parameters(m_nh_private))
{
}

} // namespace pet::control


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_node");
    google::InitGoogleLogging(argv[0]);

    auto mpc_node = pet::control::MpcNode{};

    ros::spin();

    return 0;
}
