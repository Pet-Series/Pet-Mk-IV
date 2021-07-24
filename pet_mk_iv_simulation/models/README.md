# README: **/pet_mk_iv_simulation/models/** #

ROS Package folder in the https://github.com/kullken/Pet-Mk-IV repository.</br>

## **What is this folder for?** ##
* Libraray of models used by Gazebo

## **Note!**
* Must add this model path into the "/home/sk/.gazebo/gui.ini" file.

## **Local model paths comes from**

* The GAZEBO_MODEL_PATH environment variable, which is usually set by sourcing the setup.sh script , but is often incremented in several other ways.
* The [model_paths] item in ~/.gazebo/gui.ini 
(this is how ~/model_editor_models and ~/building_editor_models are usually added)
* The ~/.gazebo/models path is always checked
* Online paths come from:
   The GAZEBO_MODEL_DATABASE_URI environment variable, which defaults to http://models.gazebosim.org
* If you have Fuel Tools enabled, the list of servers in ~/.ignition/fuel/config.yaml