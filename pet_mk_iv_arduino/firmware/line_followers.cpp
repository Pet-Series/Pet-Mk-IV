#include "line_followers.h"

#include <Arduino.h>

#include "ros.h"

#include "pet_mk_iv_msgs/TripleBoolean.h"

constexpr unsigned int kLineFollowerLeftPin      = 2;
constexpr unsigned int kLineFollowerMiddlePin    = 3;
constexpr unsigned int kLineFollowerRightPin     = 4;

extern pet::ros::NodeHandle nh;
pet_mk_iv_msgs::TripleBoolean lineFollowerMsg;
ros::Publisher lineFollowerPub("line_followers", &lineFollowerMsg);

void lineFollowerSetup()
{
    pinMode(kLineFollowerLeftPin, INPUT);
    pinMode(kLineFollowerMiddlePin, INPUT);
    pinMode(kLineFollowerRightPin, INPUT);

    lineFollowerMsg.header.frame_id = "line_followers";
    nh.advertise(lineFollowerPub);
}

void lineFollowerUpdate()
{
    lineFollowerMsg.header.stamp = nh.now();

    lineFollowerMsg.left = digitalRead(kLineFollowerLeftPin);
    lineFollowerMsg.middle = digitalRead(kLineFollowerMiddlePin);
    lineFollowerMsg.right = digitalRead(kLineFollowerRightPin);

    lineFollowerPub.publish(&lineFollowerMsg);
}