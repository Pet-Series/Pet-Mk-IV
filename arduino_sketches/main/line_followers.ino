#define lineFollowerLeftPin 2
#define lineFollowerMiddlePin 3
#define lineFollowerRightPin 4

volatile bool lineFollowerLeftValue;
volatile bool lineFollowerMiddleValue;
volatile bool lineFollowerRightValue;

void lineFollowerSetup() {
  pinMode(lineFollowerLeftPin, INPUT);
  pinMode(lineFollowerMiddlePin, INPUT);
  pinMode(lineFollowerRightPin, INPUT);

//  enableInterrupt(lineFollowerLeftPin, callbackLineFollowerLeft, CHANGE);
//  enableInterrupt(lineFollowerMiddlePin, callbackLineFollowerMiddle, CHANGE);
//  enableInterrupt(lineFollowerRightPin, callbackLineFollowerRight, CHANGE);
//
//  lineFollowerLeftValue = digitalRead(lineFollowerLeftPin);
//  lineFollowerMiddleValue = digitalRead(lineFollowerMiddlePin);
//  lineFollowerRightValue = digitalRead(lineFollowerRightPin);
  
  //lineFollowerMsg.header.frame_id = "line_followers";
}

void lineFollowerUpdate() {
  //lineFollowerMsg.header.stamp = nh.now();
  
  lineFollowerMsg.left = digitalRead(lineFollowerLeftPin);
  lineFollowerMsg.middle = digitalRead(lineFollowerMiddlePin);
  lineFollowerMsg.right = digitalRead(lineFollowerRightPin);

  //lineFollowerPub.publish(&lineFollowerMsg);
}

void callbackLineFollowerLeft() {
  lineFollowerLeftValue = digitalRead(lineFollowerLeftPin);
}

void callbackLineFollowerMiddle() {
  lineFollowerMiddleValue = digitalRead(lineFollowerMiddlePin);
}

void callbackLineFollowerRight() {
  lineFollowerRightValue = digitalRead(lineFollowerRightPin);
}
