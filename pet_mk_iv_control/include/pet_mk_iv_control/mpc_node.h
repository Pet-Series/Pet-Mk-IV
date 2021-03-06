#ifndef PET_CONTROL_MPC_NODE_H
#define PET_CONTROL_MPC_NODE_H

#include <ros/ros.h>

#include "pet_mk_iv_control/mpc.h"

namespace pet::control
{

class MpcNode
{
public:
    MpcNode();

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;

    Mpc m_mpc;
};

} // namespace pet::control

#endif // PET_CONTROL_MPC_NODE_H
