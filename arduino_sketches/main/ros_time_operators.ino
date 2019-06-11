
ros::Time operator+ (ros::Time& lhs, int rhs) {
  ros::Time temp = lhs;
  temp.sec += rhs;
  return temp;
}

ros::Time operator< (ros::Time& lhs, ros::Time& rhs) {
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
