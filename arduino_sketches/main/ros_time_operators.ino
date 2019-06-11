
ros::Time operator+ (const ros::Time& lhs, int rhs) {
  ros::Time temp = lhs;
  temp.sec += rhs;
  return temp;
}

ros::Time operator+ (const ros::Time& lhs, const ros::Duration rhs) {
  //TODO: add support for negative durations
  ros::Time temp = lhs;
  temp.sec += rhs.sec;
  temp.nsec += rhs.nsec;
  if (temp.nsec >= 1e9) {
    temp.sec += 1;
    temp.nsec -= 1e9;
  }
  return temp;
}

bool operator< (const ros::Time& lhs, const ros::Time& rhs) {
  if (lhs.sec < rhs.sec) {
    return true;
  } else if (lhs.sec > rhs.sec) {
    return false;
  } else if ( lhs.nsec < rhs.nsec) {
    return true;
  } else {
    return false;
  }
}
