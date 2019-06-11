#include <EnableInterrupt.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <test_pkg/TripleBoolean.h>
#include <test_pkg/TestBoolean.h>
#include <sensor_msgs/Temperature.h>

ros::NodeHandle nh;

std_msgs::String strMsg;
ros::Publisher chatterPub("chatter", &strMsg);

sensor_msgs::Temperature TESTMESSAGE;
ros::Publisher TESTPUBLISHER("TEST", &TESTMESSAGE);

test_pkg::TripleBoolean lineFollowerMsg;
//ros::Publisher lineFollowerPub("line_followers", &lineFollowerMsg);
//test_pkg::TestBoolean lineFollowerTestMsg;
//ros::Publisher lineFollowerTestPub("line_follower_test", &lineFollowerTestMsg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  while (!nh.connected()) {nh.spinOnce();};

  nh.advertise(TESTPUBLISHER);
  
  //nh.advertise(chatterPub);
  //nh.advertise(lineFollowerPub);
  //nh.advertise(lineFollowerTestPub);

  //lineFollowerSetup();

  //lineFollowerTestMsg.data = false;
}

void loop()
{
  strMsg.data = hello;
  chatterPub.publish(&strMsg);

  TESTMESSAGE.temperature = 1;

  TESTPUBLISHER.publish(&TESTMESSAGE);
  
  //lineFollowerUpdate();

  //lineFollowerTestMsg.data = !lineFollowerTestMsg.data;
  //lineFollowerTestPub.publish(&lineFollowerTestMsg);
  
  nh.spinOnce();
  delay(1000);
}
