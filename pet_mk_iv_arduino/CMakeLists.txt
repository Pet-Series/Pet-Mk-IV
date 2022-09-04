cmake_minimum_required(VERSION 3.10.2)
cmake_policy(SET CMP0048 NEW)

project(pet_mk_iv_arduino)

find_package(catkin REQUIRED COMPONENTS
  pet_mk_iv_msgs
  roscpp
  rosserial_arduino
  rosserial_client
  sensor_msgs
)

set(BUILD_SHARED_LIBS FALSE)
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
set(EXECUTABLE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})

catkin_package()

add_subdirectory(pet_mk_iv_nano)
add_subdirectory(pet_mk_iv_uno)
add_subdirectory(src)
