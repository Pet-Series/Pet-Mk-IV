# README: /pet_mk_iv_simulation/ #

ROS Packge folder in the https://github.com/Pet-Series/pet_mk_iv_simulation repository.</br>
**/TODO**: Put this package into a separate GitHub-repo.

## **What is this folder for?** ##

ROS package neccesary to be able to run robot simulation, using Gazebo.</br>
This package is only necessary if you are going to launch your robot into a simulated world.
If you only will run the physical robot - Then you do not need this package.</br> 

* Robot descriptions as .urdf/.xacro file.
* World descriptions as .world files.
* Robot launch files as .launch</br>
  - Load simulation world</br>
  - Spawn/insert/launch robot into the simulated world.</br>
  - Launch simulations nodes/scripts.
* Simulation nodes/scripts as .py files.</br>
  - Interface between the simulation enviroment and the common robot softaware.

### How do I get set up? ###

Set up for simuation in Gazebo
* PC with Ubuntu 20.04
* ROS(1) melodic</br>
  /TODO: Upgrade to ROS(1) Noetic.
* Clone of https://github.com/Pet-Series/pet_mk_iv_simulation (this repo.)

### Who do I talk to? ###

* Repo owner: "Kullken" <karl.viktor.kull@gmail.com>
* Folder contributor: "SeniorKullken" <stefan.kull@gmail.com>
