<h1 align="center">Welcome to the Pet-Mk-IV repository👋</h1>

 https://github.com/kullken/Pet-Mk-IV.git/



## Prerequisites
### Developing Enviroment
 - [X] Arduino IDE @ RPi
       https://www.arduino.cc/en/Main/Software
 - [X] Arduino IDE @ RPi Library: ROS-serial
       https://www.arduinolibraries.info/libraries/rosserial-arduino-library
 - [X] Arduino IDE @ RPi Library: New-Ping 
       https://bitbucket.org/teckel12/arduino-new-ping
       https://github.com/eliteio/Arduino_New_Ping
 - [X] Python 2 (using various ROS-API)
 - [X] cmake for buildning ROS via catkin/cmake
 - [X] cmake for buildning Arduino firmware/sketches
 - [X] This Git-repo.
 
### Target Envieroment: Main ECU
 - [X] Rasperry PI model 3
       Raspian Buster (Debian Buster 10)
 - [X] ROS Melodic Ubuntu Bionic (but we compile from source anyway on the RPi/Raspian).
 - [X] Arduino IDE (Not for the IDE itself... mainly for get/manage IDE-libraries).
 - [ ] cmake upload firmware from RPi > Ardunio system  (No use of Arduino IDE)
 
### Target Envieroment: Sub-ECU
 - [X] Arduino UNO R3 via serial/USB-cable -> RPi

### Installing
     ...

## Mechanics & Hardware
 - [X] Main ECU/CPU's: Rasperry Pi 3
       - [x] Power by: 1x USB Powerbank 1.5A
       - [x] Display: LCD 1602 via I2C interface: PC8574T
       - [x] Sensor: IMU 6050 via I2C
       - [x] Sensor: RPi Camera module 1.0
 - [X] Arduino UNO R3
       - [X] Senors: 3x HC-SR04 as Obstacle avaidance
       - [X] Senors: 3x Xyz as Line Follower
       - [x] Engine controller: L298N
 - [X] Chassie: Zumo with belt drive
       - [x] Power power: 6x AAA-batteries (4+2 config => 6*1,2V=7,2V)

### External references
- SeniorKullkens OneNote: https://1drv.ms/u/s!Aq6kdS_u7ZDZ5REO5Ms1MWFe0tBR
- ROS.org http://wiki.ros.org/
- Git command line cheat sheet https://education.github.com/git-cheat-sheet-education.pdf
- Lucid Chart Diagram https://www.lucidchart.com/invitations/accept/760f4729-2814-43cd-bd3d-829c4079b3df
