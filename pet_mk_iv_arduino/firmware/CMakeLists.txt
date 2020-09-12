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

add_library(pet_arduino_core INTERFACE)
target_sources(pet_arduino_core
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rosserial_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/timer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arduino_module.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/prgmem_string.cpp
)
target_include_directories(pet_arduino_core
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROS_LIB_DIR}
)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ROS_LIB_DIR}
  ${NEW_PING_INCLUDE_DIRS}
  ${SERVO_INCLUDE_DIRS}
  ${IRREMOTE_INCLUDE_DIRS}
)

# Not nicest solution. Would like to only add the flags for code from pet_mk_iv_arduino.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

set(UNO_SOURCE_FILES
  src/uno/modules.cpp
  src/dist_sensors.cpp
  src/engines.cpp
  src/line_sensor_module.cpp
)

set(NANO_SOURCE_FILES
  src/nano/modules.cpp
  src/light_beacon.cpp
)

# Generate Arduino Uno firmware
generate_arduino_firmware(arduino_uno
  SRCS ${UNO_SOURCE_FILES} ${NEW_PING_SOURCE_FILES}
  LIBS pet_arduino_core
  BOARD uno
  PORT /dev/ArduinoUno0
)

# Generate Arduino Nano(old) firmware
generate_arduino_firmware(arduino_nano
  SRCS ${NANO_SOURCE_FILES} ${SERVO_SOURCE_FILES}
  LIBS pet_arduino_core
  BOARD nano328
  PORT /dev/ArduinoNano0
)

target_compile_definitions(arduino_uno
  PUBLIC
    "ARDUINO_ARCH_AVR"
)

target_compile_definitions(arduino_nano
  PUBLIC
    "ARDUINO_ARCH_AVR"
)

