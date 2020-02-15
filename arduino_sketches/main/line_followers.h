#ifndef _PET_LINEFOLLOWERS_H
#define _PET_LINEFOLLOWERS_H

#include "ros.h"

#include "pet_mk_iv_msgs/TripleBoolean.h"

#define lineFollowerLeftPin 2
#define lineFollowerMiddlePin 3
#define lineFollowerRightPin 4

extern pet::ros::NodeHandle nh;
pet_mk_iv_msgs::TripleBoolean lineFollowerMsg;
ros::Publisher lineFollowerPub("line_followers", &lineFollowerMsg);

void lineFollowerSetup()
{
    pinMode(lineFollowerLeftPin, INPUT);
    pinMode(lineFollowerMiddlePin, INPUT);
    pinMode(lineFollowerRightPin, INPUT);

    lineFollowerMsg.header.frame_id = "line_followers";
    nh.advertise(lineFollowerPub);
}

void lineFollowerUpdate()
{
    lineFollowerMsg.header.stamp = nh.now();

    lineFollowerMsg.left = digitalRead(lineFollowerLeftPin);
    lineFollowerMsg.middle = digitalRead(lineFollowerMiddlePin);
    lineFollowerMsg.right = digitalRead(lineFollowerRightPin);

    lineFollowerPub.publish(&lineFollowerMsg);
}

#endif