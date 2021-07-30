# README #

Folder in the https://github.com/Pet-Series/Pet-Mk-IV repository.
Folder: ~/ros_ws/src/Pet-Mk-IV/pet_mk_iv_simulation/src/UnitTest

### What is this folder for? ###

This is a folder where I save various unit test programs/scripts...
Test scripts/codes is to be run outside ROS (This is not a ROS-package)

### How do I get set up? ###

#### To be able to run the MCU-tests ####
* PC with Arduino IDE (like a Raspberry Pi running Raspbian) 
* Arduino UNO/Nano board(MCU) 
* Various sensors, actuators and controllers connected to the Arduino board(MCU)

#### To be able to run the SBC-tests ####
* Raspberry Pi running Raspbian 
* Various sensors, actuators and controllers connected to the RPi(SBC)

#### File naming convetions ####
* "**arduino_\***..." - MCU (Arduino Uno/Nano) low level test of sensors/actuators.
* "**RPi_\***..." - SBC (Raspberry Pi running Raspbian) low level test of sensors/actuators and other peripheral components connected to the SBC.

### Who do I talk to? ###

* Repo owner: "Kullken" <karl.viktor.kull@gmail.com>
* Folder contributor: "SeniorKullken" <stefan.kull@gmail.com>

###
:warning:Be aware:warning: You might get some "*close to hardware experiences*:neckbeard:".
