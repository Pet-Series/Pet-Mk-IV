#include "ros.h"
#include <std_msgs/String.h>

extern pet::ros::NodeHandle nh;

namespace chatter
{

static const ros::Duration kPeriod(1, 0);    // sec, nsec

static std_msgs::String chatterMsg;
static ros::Publisher chatterPub("chatter", &chatterMsg);

// Will be exposed as "chatter::setup()"
void setup()   
{
    nh.advertise(chatterPub);
    chatterMsg.data = "chatter@aux";  // Dummy "Hello World" test string to publish
}

// Will be exposed as "chatter::callback()"
void callback() 
{
    chatterPub.publish(&chatterMsg);
}

} // namespace chatter