#define LeftReversePin 7
#define LeftForwardPin 8
#define LeftSpeedPin 9
#define RightSpeedPin 10
#define RightForwardPin 11
#define RightReversePin 12

const ros::Duration ENGINE_TIMEOUT(0, 0.5e9);

pet_mk_iv_msgs::EngineCommand engineCommandMsg;
ros::Subscriber<pet_mk_iv_msgs::EngineCommand> engineCommandSub("engine_command", &engineCommandCb);

void enginesSetup() {
  pinMode(LeftReversePin, OUTPUT);
  pinMode(LeftForwardPin, OUTPUT);
  pinMode(LeftSpeedPin, OUTPUT);
  pinMode(RightSpeedPin, OUTPUT);
  pinMode(RightForwardPin, OUTPUT);
  pinMode(RightReversePin, OUTPUT);

  digitalWrite(LeftReversePin, LOW);
  digitalWrite(LeftForwardPin, LOW);
  digitalWrite(LeftSpeedPin, LOW);
  digitalWrite(RightSpeedPin, LOW);
  digitalWrite(RightForwardPin, LOW);
  digitalWrite(RightReversePin, LOW);
  
  nh.subscribe(engineCommandSub);
}


void enginesUpdate() {
  if (nh.now() < engineCommandMsg.header.stamp + ENGINE_TIMEOUT) { //engineCommandMsg.header.stamp.toSec() + 1.0 > nh.now().toSec()
    if (engineCommandMsg.left_direction == engineCommandMsg.FORWARD) {
      digitalWrite(LeftReversePin, LOW);
      digitalWrite(LeftForwardPin, HIGH);
      analogWrite(LeftSpeedPin, engineCommandMsg.left_pwm);
    }
    else if (engineCommandMsg.left_direction == engineCommandMsg.BACKWARD) {
      digitalWrite(LeftForwardPin, LOW);
      digitalWrite(LeftReversePin, HIGH);
      analogWrite(LeftSpeedPin, engineCommandMsg.left_pwm);
    }
    else {
      digitalWrite(LeftReversePin, LOW);
      digitalWrite(LeftForwardPin, LOW);
      analogWrite(LeftSpeedPin, 0);
    }
    
    if (engineCommandMsg.right_direction == engineCommandMsg.FORWARD) {
      digitalWrite(RightReversePin, LOW);
      digitalWrite(RightForwardPin, HIGH);
      analogWrite(RightSpeedPin, engineCommandMsg.right_pwm);
    }
    else if (engineCommandMsg.right_direction == engineCommandMsg.BACKWARD) {
      digitalWrite(RightForwardPin, LOW);
      digitalWrite(RightReversePin, HIGH);
      analogWrite(RightSpeedPin, engineCommandMsg.right_pwm);
    }
    else {
      digitalWrite(RightReversePin, LOW);
      digitalWrite(RightForwardPin, LOW);
      analogWrite(RightSpeedPin, 0);
    }
    
  }
  else {
    digitalWrite(LeftReversePin, LOW);
    digitalWrite(LeftForwardPin, LOW);
    digitalWrite(LeftSpeedPin, LOW);
    digitalWrite(RightSpeedPin, LOW);
    digitalWrite(RightForwardPin, LOW);
    digitalWrite(RightReversePin, LOW);
  }
}

void engineCommandCb (const pet_mk_iv_msgs::EngineCommand & msg) {
  engineCommandMsg = msg;
  
  }
