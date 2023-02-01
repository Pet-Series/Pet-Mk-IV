# README #

Folder in the https://github.com/kullken/Pet-Mk-IV repository.
Folder: ~/ros_ws/src/Pet-Mk-IV/pet_mk_iv_simulation/src/diagnostics

### What is this folder for? ###

This is a folder where I save various diagnostics, POC and test tools...
Test scripts/codes is to be run outside ROS1/ROS2 (This is not a ROS-package)

### How do I get set up? ###

#### To be able to run the MCU-tests ####
* PC with Arduino IDE (like a Raspberry Pi running Raspbian) 
* Arduino UNO/Nano board(MCU) 
* Various sensors, actuators and controllers connected to the Arduino board(MCU)

#### To be able to run the SBC-tests ####
* Raspberry Pi running Raspbian OR Ubuntu
* Various sensors, actuators and controllers connected to the SBC(RPi) or the MCU(Arduino)

#### File naming convetions ####
* "**arduino_\***..." - MCU (Arduino Uno/Nano) low level test/diagnostics of sensors/actuators.
* "**RPi_\***..." - SBC (Raspberry Pi running Raspbian) low level test/diagnostics of sensors/actuators and other peripheral components connected to the SBC.

### Who do I talk to? ###

* Repo owner: "SeniorKullken" <stefan.kull@gmail.com>
* Folder contributor: "Kullken" <karl.viktor.kull@gmail.com>

###
:warning:Be aware:warning: You might get some "*close to hardware experiences*:neckbeard:".
