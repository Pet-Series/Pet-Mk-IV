<h1 align="center">Welcome to the Pet-Mk-IV repository👋</h1>

 https://github.com/kullken/Pet-Mk-IV.git/



# Prerequisites
### Developing Environment
 - [X] Visual Studio Code@ Local PC
   - https://code.visualstudio.com/
 - [X] Arduino IDE @ RPi target system.
   - https://www.arduino.cc/en/Main/Software
 - [X] Arduino IDE @ RPi - Library: ROS-serial
   https://www.arduinolibraries.info/libraries/rosserial-arduino-library
 - [X] Arduino IDE @ RPi - Library: New-Ping 
   - https://bitbucket.org/teckel12/arduino-new-ping
   - https://github.com/eliteio/Arduino_New_Ping
 - [X] Python 2 (using various ROS-API)
 - [X] cmake for building ROS via catkin/cmake
 - [X] cmake for building Arduino firmware/sketches
 - [X] This Git-repo.
 
### Target Environment: Main ECU
 - [x] Raspberry PI model 3
       Raspbian Buster (Debian Buster 10)
 - [X] ROS Melodic Ubuntu Bionic (but we compile from source anyway on the RPi/Raspbian).
 - [X] Arduino IDE (Not for the IDE itself... mainly for get/manage IDE-libraries).
 - [ ] cmake upload firmware from RPi > Arduino system  (No use of Arduino IDE)
 
### Target Environment: Sub-ECU
 - [X] Arduino UNO R3 via serial/USB-cable -> RPi

### Installing
     ...

## Mechanics & Hardware
- [X] **Main ECU/CPU's:** Raspberry Pi 3
  - [X] **Power by:** 1x USB Power bank 1.5A
  - [x] **Display:** LCD 1602 via I2C interface: PC8574T
  - [x] **Sensor:** IMU GY-521/MPU6050(6 axis) via I2C
  - [x] **Sensor:** Camera RPi Camera module 1.0
- [X] **Arduino UNO R3**
  - [x] **Sensor:** 3x HC-SR04 ultrasonic ranging sensor for Obstacle detection
  - [x] **Sensor:** 3x CTRT5000 Infrared sensors for tracing
  - [x] **Engine controller:** L298N Dual H Bridge Step/DC-motor controller
- [X] **Chassis: Zumo with belt drive**
  - [x] Power: 6x AAA-batteries (4+2 config => 6*1,2V=7,2V)

### Images
- <img src="/Project_stuff/Images/2019-09-09_122618.jpg" width="350px">|<img src="/Project_stuff/Images/2019-09-09_122725.jpg" width="350px">
- [More images: /Project_stuff/Images](/Project_stuff/Images)

# External references
- ROS.org
  - http://wiki.ros.org/
- GitHub cheat sheets
  - https://help.github.com/en/github/writing-on-github/basic-writing-and-formatting-syntax
  - https://www.webfx.com/tools/emoji-cheat-sheet/
