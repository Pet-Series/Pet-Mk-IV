cmake_minimum_required(VERSION 3.10.2)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

# NewPing Arduino library
set(NEW_PING_SOURCE_DIR $ENV{HOME}/Arduino/libraries/NewPing/src)
set(NEW_PING_INCLUDE_DIRS ${NEW_PING_SOURCE_DIR})
set(NEW_PING_SOURCE_FILES
  ${NEW_PING_SOURCE_DIR}/NewPing.cpp
)

# Servo Arduino library
set(SERVO_SOURCE_DIR /opt/arduino-1.8.10/libraries/Servo/src)
set(SERVO_INCLUDE_DIRS
  ${SERVO_SOURCE_DIR}
  ${SERVO_SOURCE_DIR}/avr
)
set(SERVO_SOURCE_FILES
  ${SERVO_SOURCE_DIR}/avr/Servo.cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ROS_LIB_DIR}
  ${NEW_PING_INCLUDE_DIRS}
  ${SERVO_INCLUDE_DIRS}
)

# Not nicest solution. Would like to only add the flags for code from pet_mk_iv_arduino.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

set(${PROJECT}_SOURCE_FILES
  src/main.cpp
  src/rosserial_node.cpp
  src/timer.cpp
  src/arduino_module.cpp
  src/dist_sensors.cpp
  src/engines.cpp
  src/line_followers.cpp
)

# Generate Arduino Uno firmware
generate_arduino_firmware(hello
  SRCS ${${PROJECT}_SOURCE_FILES} ${NEW_PING_SOURCE_FILES} ${SERVO_SOURCE_FILES}
  BOARD uno
  PORT /dev/ArduinoUno0
)

target_compile_definitions(hello
  PUBLIC
    "ARDUINO_ARCH_AVR"
)

