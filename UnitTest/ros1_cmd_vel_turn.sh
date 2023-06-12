#!/bin/bash
# ROS1 - Test bash script
echo "Turn"
source ~/ros_ws/devel/setup.bash 
rostopic pub -1 /cmd_vel geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 0.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5"
echo "Done"