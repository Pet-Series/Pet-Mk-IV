cmake_minimum_required(VERSION 2.8.3)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/firmware
  ${ROS_LIB_DIR}
)

# Not nicest solution. Would like to only add the flags for code from pet_mk_iv_arduino.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

set(${PROJECT}_SOURCE_FILES
  main.cpp
  ros_time_operators.cpp
)

set(EXTERNAL_SOURCE_FILES
  ${ROS_LIB_DIR}/time.cpp
  ${ROS_LIB_DIR}/duration.cpp
)

generate_arduino_firmware(arduino
  SRCS ${${PROJECT}_SOURCE_FILES} ${EXTERNAL_SOURCE_FILES}
  BOARD nano328
  # PROGRAMMER  
  PORT /dev/ArduinoNano0
)
