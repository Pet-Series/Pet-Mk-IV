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
 
### Target Environment: Main ECU/MCU (Single Board Computer, SBC) 
 - [x] Raspberry PI model 3
       Raspbian Buster (Debian Buster 10)
 - [X] ROS Melodic Ubuntu Bionic (but we compile from source anyway on the RPi/Raspbian).
 - [X] Arduino IDE (Not for the IDE itself... mainly for get/manage IDE-libraries).
 - [X] cmake upload firmware from RPi > Arduino system  (No use of Arduino IDE)
 
### Target Environment: Sub-ECU #1
 - [X] Arduino UNO R3 via serial/USB-cable -> RPi
 - [X] Firmware <- https://github.com/kullken/pet_mcu_base

### Target Environment: Sub-ECU #2
 - [X] Arduino Nano via serial/USB-cable -> RPi
 - [X] Firmware <- https://github.com/kullken/pet_mcu_base
 
### Installing
     ...

## Mechanics & Hardware
- [X] **Main ECU/CPU's:** Raspberry Pi 3
  - [X] **Power by:** 1x USB Power bank 1.5A
  - [x] **Display:** LCD 1602 via I2C interface: PC8574T
  - [x] **Sensor:** IMU GY-521/MPU6050(6 axis) via I2C
  - [x] **Sensor:** Camera RPi Camera module 1.0
- [X] **Sub-ECU #1:** Arduino UNO R3
  - [X] **Power by:** Via RPi (1x USB Power bank 1.5A)
  - [x] **Sensor:** 3x HC-SR04 ultrasonic ranging sensor for Obstacle detection
  - [x] **Sensor:** 3x CTRT5000 Infrared sensors for tracing
  - [x] **Engine controller:** L298N Dual H Bridge Step/DC-motor controller
- [X] **Auxiliary-ECU #2:** Arduino Nano
  - [X] **Power by:** Chassis battery pack
  - [ ] **Sensor:** 1x KY-022 (1838T) Infrared(IR) Receiver 
  - [ ] **Flash light** Flash/Strobe light
- [X] **Chassis: Zumo with belt drive**
  - [x] **Power source:** 6x AAA-batteries (4+2 config => 6*1,2V=7,2V)
  - [X] **Engines:** 2x Electrical engines (Controlled via "L298N Dual H Bridge"@Sub-ECU #1: Arduino UNO R3)

### Images
- <img src="/Project_stuff/Images/2019-09-09_122618.jpg" width="350px">|<img src="/Project_stuff/Images/2019-09-09_122725.jpg" width="350px">
- [More images: /Project_stuff/Images](/Project_stuff/Images)

# External references
- ROS.org
  - http://wiki.ros.org/
- GitHub cheat sheets
  - https://guides.github.com/features/mastering-markdown/
  - https://help.github.com/en/github/writing-on-github/basic-writing-and-formatting-syntax
  - https://www.webfx.com/tools/emoji-cheat-sheet/

# Glossary
**Buzzword** | **My interpretation**
---------| -----------------
**ECU**	| [Electronic Control Unit](https://en.wikipedia.org/wiki/Electronic_control_unit)
**IDE** | [Integrated_Development_Environment](https://en.wikipedia.org/wiki/Integrated_development_environment)
**MCU** | [Micro Control Unit](https://en.wikipedia.org/wiki/Microcontroller)
**Pet** | Not an abbreviation. Simply names of our virtual pets (aka. Robots)
**SBC** | [Single Board Computer](https://en.wikipedia.org/wiki/Single-board_computer)
