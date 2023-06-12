#!/bin/bash
# ROS1 - Test bash script
echo "Twist"
source ~/ros_ws/devel/setup.bash 
rostopic pub --once /cmd_vel geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 1.0"
echo "Done"