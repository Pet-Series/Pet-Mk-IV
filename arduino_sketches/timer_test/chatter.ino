
#include <std_msgs/String.h>

std_msgs::String chatterMsg;
ros::Publisher chatterPub("chatter", &chatterMsg);

char chatterData[] = "chatter";

void chatterSetup() {
  nh.advertise(chatterPub);
}

void chatterUpdate() {
  chatterMsg.data = chatterData;
  chatterPub.publish(&chatterMsg);
}
