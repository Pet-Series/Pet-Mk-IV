#ifndef _PET_CHATTER_H
#define _PET_CHATTER_H

#include "ros.h"
#include <std_msgs/String.h>

extern pet::ros::NodeHandle nh;
std_msgs::String chatterMsg;
ros::Publisher chatterPub("chatter", &chatterMsg);

void chatterSetup()
{
    chatterMsg.data = "chatter";
    nh.advertise(chatterPub);
}

void chatterUpdate()
{
    chatterPub.publish(&chatterMsg);
}

#endif