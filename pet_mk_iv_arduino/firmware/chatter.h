#ifndef _PET_CHATTER_H
#define _PET_CHATTER_H

#include "ros.h"
#include <std_msgs/String.h>

extern pet::ros::NodeHandle nh;
std_msgs::String chatterMsg;
ros::Publisher chatterPub("chatter", &chatterMsg);

//const char chatterData[8] = "chatter";

// void chatterSetup(ros::NodeHandle& nh)
void chatterSetup()
{
    chatterMsg.data = "chatter";
    nh.advertise(chatterPub);
}

void chatterUpdate()
{
    //chatterMsg.data = chatterData;
    chatterPub.publish(&chatterMsg);
}

// class chatter
// {
// private:
//     ros::NodeHandle m_nh;
//     ros::Publisher m_pub;
//     std_msgs::String m_msg{};

// public:
//     chatter(ros::NodeHandle& nh) : m_nh(nh), {}
// };



#endif