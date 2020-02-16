cmake_minimum_required(VERSION 2.8.3)

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
# add_definitions(-DUSB_CON)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

# NewPing Arduino library
set(NEW_PING_INCLUDE_DIR /home/pi/Arduino/libraries/NewPing/src)
set(NEW_PING_SOURCE_DIR /home/pi/Arduino/libraries/NewPing/src)
set(NEW_PING_SOURCE_FILES 
  ${NEW_PING_SOURCE_DIR}/NewPing.cpp
)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  ${ROS_LIB_DIR}
  ${NEW_PING_INCLUDE_DIR}
)

set(${PROJECT}_SOURCE_FILES
  main.cpp
  dist_sensors.cpp
  engines.cpp
  ros_time_operators.cpp
)

set(EXTERNAL_SOURCE_FILES
  ${ROS_LIB_DIR}/time.cpp
  ${ROS_LIB_DIR}/duration.cpp
  ${NEW_PING_SOURCE_FILES}
)

generate_arduino_firmware(hello
  SRCS ${${PROJECT}_SOURCE_FILES} ${EXTERNAL_SOURCE_FILES}
  BOARD uno
  PORT /dev/ttyUSB0
)