cmake_minimum_required(VERSION 3.10.2)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

# IRremote Arduino library
set(IRREMOTE_INCLUDE_DIR $ENV{HOME}/Arduino/libraries/IRremote)
set(IRREMOTE_SOURCE_DIR $ENV{HOME}/Arduino/libraries/IRremote)

set(IRREMOTE_SOURCE_FILES 
  # Source (.cpp) files from IRremote library might need to be declared here.
  ${IRREMOTE_SOURCE_DIR}/IRremote.cpp
  ${IRREMOTE_SOURCE_DIR}/irRecv.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ROS_LIB_DIR}
  ${IRREMOTE_INCLUDE_DIR}
)

# Not nicest solution. Would like to only add the flags for code from pet_mk_iv_arduino.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

set(${PROJECT}_SOURCE_FILES
  src/main.cpp
  src/chatter.cpp
  src/irremote.cpp
)

set(EXTERNAL_SOURCE_FILES
  ${ROS_LIB_DIR}/time.cpp
  ${ROS_LIB_DIR}/duration.cpp
)

generate_arduino_firmware(arduino
  SRCS ${${PROJECT}_SOURCE_FILES} ${EXTERNAL_SOURCE_FILES}
  BOARD nano328
  PORT /dev/ArduinoNano0
)
