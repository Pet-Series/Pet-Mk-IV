//Confirmed working with Arduino IDE 1.8.9 
//Dependencies: EnableInterrupt, Rosserial Arduino Library
//Custom ros packages needs to be processed, guide on git.

#include <EnableInterrupt.h>

#include <ros.h>
#include <ros/time.h>


#include "pet_mk_iv_msgs/EngineCommand.h"
#include "pet_mk_iv_msgs/TripleBoolean.h"

ros::NodeHandle nh;

void setup()
{
  nh.initNode();

  lineFollowerSetup();
  enginesSetup();
  
}

void loop()
{
  
  lineFollowerUpdate();
  enginesUpdate();
  
  nh.spinOnce();
  delay(1000);
}
