cmake_minimum_required(VERSION 2.8.3)
project(pet_mk_iv_arduino)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

# rosserial_generate_ros_lib(
#   PACKAGE rosserial_arduino
#   SCRIPT make_libraries.py
# )

# rosserial_configure_client(
#   DIRECTORY firmware
#   TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
# )

# rosserial_add_client_target(firmware hello ALL)
# rosserial_add_client_target(firmware hello-upload)