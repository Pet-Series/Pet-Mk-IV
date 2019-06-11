#define lineFollowerLeftPin 2
#define lineFollowerMiddlePin 3
#define lineFollowerRightPin 4


void lineFollowerSetup() {
  pinMode(lineFollowerLeftPin, INPUT);
  pinMode(lineFollowerMiddlePin, INPUT);
  pinMode(lineFollowerRightPin, INPUT);
  
  //lineFollowerMsg.header.frame_id = "line_followers";
}

void lineFollowerUpdate() {
  //lineFollowerMsg.header.stamp = nh.now();
  
  lineFollowerMsg.left = digitalRead(lineFollowerLeftPin);
  lineFollowerMsg.middle = digitalRead(lineFollowerMiddlePin);
  lineFollowerMsg.right = digitalRead(lineFollowerRightPin);

  lineFollowerPub.publish(&lineFollowerMsg);
}
