# README: /pet_mk_iv_description/urdf/ #

Sub-folder in the https://github.com/Pet-Series/Pet-Mk-IV repository.</br>

## **What is this folder for?** ###

Robot Descriptions using the .sdf, .urdf or .xacro syntax.
Used, among other things, by Gazebo (3D Visual/Collision), RViz (3D Visual).  

## **Robot Chassis & Body/Frame**
<table style="width:100%">
  <tr>
    <th>Screen shoot</th>
    <th>.urdf/.xacro /.sdf</th>
  </tr>
  <tr>
    <td>chassis_zumo.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-chassis_PololuZumo_link_tree.png" width="350px">
    </td>
    <td>Pololu Zumo Chassis - Link tree</br>
        Chassis with battery compartment & hatch + two engines.</br>
        `base_link` and `body_base_link`
 
```xml
<xacro:macro name="chassis_zumo" params="name" >
  ...
  <link name='${name}'>
  ...
  <visual name='Zumo_Chassis_visual'>
  ...
  <collision name='Zumo_Chassis_visual_collision'>
```

   </td>
  </tr>
  <tr>
    <td>wheel.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-wheel.urdf.xacro.png" width="350px"></td>
    <td>Pololu Zumo Chassis wheel - Link tree</br>
        Belt drive wheels.</br>
        `xxx_wheel`-link relative to `base_link`

```xml
<xacro:macro name="wheel" params="wheel_prefix origin_xyz origin_rpy">
  ...
  <link name="${wheel_prefix}_wheel">
    ...
    <visual name="${wheel_prefix}_wheel_visual">
    <visual name="${wheel_prefix}_wheel_visual_2">
    ...
    <collision name="${wheel_prefix}_wheel_collision">
```

   </td>
  </tr>
  <tr>
    <td>pet_mk_iv.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-chassis_PololuZumo_and_wheel_link_tree.png" width="350px"></td>
    <td> Showing Chassis & Wheels</br>
        `body_base_link` relative to `base_link`
    
```xml
<link name='body_base_link'>
  ...
  <link name='base_link_inertial'>
```          
   </tr>
  <tr>
    <td>pet_mk_iv.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-body_link_visual.png" width="350px"></td>
    <td> Pet Mk. IV body - Visual goemtry</br>
        Visual geometry for the robot upper-/lower deck.</br>
        
```xml
<link name='body_base_link'>
  ...
  <visual name='Pet_Mk_IV_body_visual'>
  ...
  <visual name='Pet_Mk_IV_body_grip_link_visual'>
```
   </td>
  </tr>
  <tr>
    <td>pet_mk_iv.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-body_link_collision.png" width="350px">
    </td>
    <td>Pet Mk.IV body - Collision geometry</br>
        Collision geometry for the robot, used by simulator applications.</br>

```xml
<link name='body_base_link'>
  ...
  <collision name='Pet_Mk_IV_body_grip_collision'>
  ...
  <collision name='Pet_Mk_IV_body_upper_deck_collision'>
  ...
  <collision name='Pet_Mk_IV_body_lower_deck_collision'>
```
   </td>
  </tr>
  <tr>
    <td>visual_raspberryPi3B_sbc.urdf.xacro.png<br/>
        <img src="/doc/pet_mk_iv-visual_raspberryPi3B_sbc.urdf.xacro.png" width="350px">
    </td>
    <td>Raspberry PI3B - visual geometry**</br>
        `name`-link  relative to `base_link`
        
```xml
<xacro:macro name="visual_raspberryPi3B_sbc" params="name origin_xyz origin_rpy">
  ...
  <link name="${name}">
    ...
    <visual name="power_led_visual">
```
   </td>
  </tr>
  <tr>
    <td>visual_LCD1602.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-visual_LCD1602.urdf.xacro.png" width="350px">
    </td>
    <td>LCD1602 PCB with I2C-piggy back**</br>
        `name`-link  relative to `base_link`
        
```xml
<xacro:macro name="visual_LCD1602" params="display_name origin_xyz origin_rpy">
  ...
  <link name="${name}">
```
   </td>
  </tr>
</table>

## **Robot Sensors**

<table style="width:100%">
  <tr>
    <th>Screen shoot</th>
    <th>.urdf/.xacro comments/.sdf</th>
  </tr>
  <tr>
    <td>sensor_hc_sr04_sonarRange.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-sensor_hc_sr04_sonarRange.urdf.xacro.png" width="350px">
    </td>
    <td>HC-SR04 distance/range sensor as .urdf/.xacro<br/>
        Used as obstacle detectors. Measuring distance/range to obstacles."
    </td>
  </tr>
  <tr>
    <td>sensor_fc_123_lineDetector.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-sensor_fc_123_lineDetector.urdf.xacro.png" width="350px"></td>
    <td>FC-123 sensor as .urdf/.xacro<br/>
        Break out board for the CTR5000 IR-reflective sensor-chip<br/>
        Used as downwards line detector (aka. "Line follower")
    </td>
  </tr>
    <tr>
    <td>sensor_RPi_cameraV2.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-sensor_RPi_cameraV2.urdf.xacro.png" width="350px"></td>
    <td>Raspberry PI Camera V2 sensor as .urdf/.xacro
        Used for optical detection and/or FPS-view feedback for remote driver.
    </td>
  </tr>
  <tr>
    <td>sensor_mpu6050_imu.urdf.xacro<br/>
        <img src="/doc/pet_mk_iv-sensor_mpu6050_imu.urdf.xacro.png" width="350px"></td>
    <td>GY-512 IMU sensor as .urdf/.xacro<br/>
        Break out board for the MPU6050 IMU sensor-chip<br/>
        Used as movement/acceleration detector along X, Y ans Z axis(aka. "Line follower")</td>
  </tr>
</table>

<hr/>

## **Nice to have - External references** ##
* http://wiki.ros.org/xacro
* http://wiki.ros.org/urdf/XML/robot
* http://sdformat.org/

<hr/>
#robots #robotics #ros #urdf #rviz #gazebo #simulation #pololu #zumo
