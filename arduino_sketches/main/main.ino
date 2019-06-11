//Confirmed working with Arduino IDE 1.8.9 
//Dependencies: EnableInterrupt, Rosserial Arduino Library
//Custom ros packages needs to be processed, guide on git.

#include <EnableInterrupt.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <test_pkg/TripleBoolean.h>

ros::NodeHandle nh;

std_msgs::String strMsg;
ros::Publisher chatterPub("chatter", &strMsg);

test_pkg::TripleBoolean lineFollowerMsg;
ros::Publisher lineFollowerPub("line_followers", &lineFollowerMsg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  
  nh.advertise(chatterPub);
  nh.advertise(lineFollowerPub);

  lineFollowerSetup();

}

void loop()
{
  strMsg.data = hello;
  chatterPub.publish(&strMsg);
  
  lineFollowerUpdate();
  
  nh.spinOnce();
  delay(1000);
}
