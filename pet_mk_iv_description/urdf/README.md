# README: /pet_mk_iv_description/urdf/ #

ROS Package folder in the https://github.com/kullken/Pet-Mk-IV repository.</br>

## **What is this folder for?** ###

Robot Descriptions using the .sdf, .urdf or .xacro syntax.
Used, among other things, by Gazebo (3D Visual/Collision), RViz (3D Visual).  
* http://wiki.ros.org/xacro
* http://wiki.ros.org/urdf/XML/robot
* http://sdformat.org/

## **Robot Chassis & Body/Frame**
<table style="width:100%;background-color:#dddddd">
  <tr>
    <th>Screen shoot</th>
    <th>.urdf/.xacro /.sdf</th>
  </tr>
  <tr>
    <td>pet_mk_iv.urdf.xacro<br/>
        <img src="/Project_stuff/Images/chassis_zumo.urdf.xacro.png" width="350px">
    </td>
    <td>**Pololu Zumo Chassis**</br>
        Chassis with battery compartment & hatch + two engines.</br>
        `base_link` and `body_base_link`
    </td>
  </tr>
  <tr>
    <td>wheel.urdf.xacro<br/>
        <img src="/Project_stuff/Images/wheel.urdf.xacro.png" width="350px"></td>
    <td>**Pololu Zumo Chassis wheels**</br>
        Belt drive wheels.</br>
        `base_link` and `xxx_wheel`-link
    </td>
  </tr>
  <tr>
    <td>chassis_zumo.urdf.xacro<br/>
        <img src="/Project_stuff/Images/chassis_zumo.and.wheels.png" width="350px"></td>
    <td> **Showing Chassis & Wheels.**</br>
        `base_link` and `body_base_link`
  </tr>
  <tr>
    <td>chassis_zumo.urdf.xacro<br/>
        <img src="/Project_stuff/Images/body_link_visual.png" width="350px"></td>
    <td> **Pet Mk. IV body - Visual goemtry**</br>
        Visual geometry for the robot
    </td>
  </tr>
  <tr>
    <td>chassis_zumo.urdf.xacro<br/>
        <img src="/Project_stuff/Images/body_link_collision.png" width="350px">
    </td>
    <td>**Pet Mk. IV body - Collision goemtry**</br>
        Collision geometry for the robor, used by simulator applications.
    </td>
  </tr>
  <tr>
    <td>visual_raspberryPi3B_sbc.urdf.xacro.png<br/>
        <img src="/Project_stuff/Images/visual_raspberryPi3B_sbc.urdf.xacro.png" width="350px">
    </td>
    <td>**Pet Mk. IV body - Collision goemtry**</br>
        Visual geometry.</br>
        `base_link` and `gemoetry`-link
    </td>
  </tr>
</table>

## **Robot Sensors**

<table style="width:100%;background-color:#dddddd">
  <tr>
    <th>Screen shoot</th>
    <th>.urdf/.xacro comments/.sdf</th>
  </tr>
  <tr>
    <td>sensor_hc_sr04_sonarRange.urdf.xacro<br/>
        <img src="/Project_stuff/Images/sensor_hc_sr04_sonarRange.urdf.xacro.png" width="350px">
    </td>
    <td>HC-SR04 distance/range sensor as .urdf/.xacro<br/>
        Used as obstacle detectors. Measuring distance/range to obstacles."
    </td>
  </tr>
  <tr>
    <td>sensor_fc_123_lineDetector.urdf.xacro<br/>
        <img src="/Project_stuff/Images/sensor_fc_123_lineDetector.urdf.xacro.png" width="350px"></td>
    <td>FC-123 sensor as .urdf/.xacro<br/>
        Break out board for the CTR5000 IR-reflective sensor-chip<br/>
        Used as downwards line detector (aka. "Line follower")
    </td>
  </tr>
    <tr>
    <td>sensor_RPi_cameraV2.urdf.xacro<br/>
        <img src="/Project_stuff/Images/sensor_RPi_cameraV2.urdf.xacro.png" width="350px"></td>
    <td>Raspberry PI Camera V2 sensor as .urdf/.xacro
        Used for optical detection and/or FPS-view feedback for remote driver.
     </td>
  </tr>
  <tr>
    <td>sensor_mpu6050_imu.urdf.xacro<br/>
        <img src="/Project_stuff/Images/sensor_mpu6050_imu.urdf.xacro.png" width="350px"></td>
    <td>GY-512 IMU sensor as .urdf/.xacro<br/>
        Break out board for the MPU6050 IMU sensor-chip<br/>
        Used as movement/acceleration detector along X, Y ans Z axis(aka. "Line follower")</td>
  </tr>
</table>
<hr />
        #robots #robotics #ros #urdf #rviz #gazebo #simulation #pololu #zumo
