cmake_minimum_required(VERSION 3.10.2)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino/avr)

# NewPing Arduino library
set(NEW_PING_SOURCE_DIR $ENV{HOME}/Arduino/libraries/NewPing/src)
set(NEW_PING_INCLUDE_DIRS ${NEW_PING_SOURCE_DIR})
set(NEW_PING_SOURCE_FILES
  ${NEW_PING_SOURCE_DIR}/NewPing.cpp
)

add_library(new_ping INTERFACE)
target_sources(new_ping
  INTERFACE
    ${NEW_PING_SOURCE_FILES}
)
target_include_directories(new_ping
  INTERFACE
    ${NEW_PING_INCLUDE_DIRS}
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

add_library(servo INTERFACE)
target_sources(servo
  INTERFACE
    ${SERVO_SOURCE_FILES}
)
target_include_directories(servo
  INTERFACE
    ${SERVO_INCLUDE_DIRS}
)

# IRremote Arduino library
set(IRREMOTE_INCLUDE_DIRS $ENV{HOME}/Arduino/libraries/IRremote)
set(IRREMOTE_SOURCE_DIR $ENV{HOME}/Arduino/libraries/IRremote)
set(IRREMOTE_SOURCE_FILES
  ${IRREMOTE_SOURCE_DIR}/IRremote.cpp
  ${IRREMOTE_SOURCE_DIR}/irRecv.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_NEC.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Sony.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Sanyo.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Mitsubishi.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_RC5_RC6.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Panasonic.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_LG.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_JVC.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Samsung.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Whynter.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Aiwa.cpp
  ${IRREMOTE_SOURCE_DIR}/ir_Denon.cpp
)

add_library(irremote INTERFACE)
target_sources(irremote
  INTERFACE
    ${IRREMOTE_SOURCE_FILES}
)
target_include_directories(irremote
  INTERFACE
    ${IRREMOTE_INCLUDE_DIRS}
)

add_library(pet_arduino_core INTERFACE)
target_sources(pet_arduino_core
  INTERFACE
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
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

add_library(pet_arduino_uno INTERFACE)
target_sources(pet_arduino_uno
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uno/modules.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dist_sensors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/engines.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/line_sensor_module.cpp
)
target_include_directories(pet_arduino_uno
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROS_LIB_DIR}
)
target_link_libraries(pet_arduino_uno
  INTERFACE
    pet_arduino_core
    new_ping
)

add_library(pet_arduino_nano INTERFACE)
target_sources(pet_arduino_nano
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/nano/modules.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ir_remote_module.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/light_beacon.cpp
)
target_include_directories(pet_arduino_nano
  INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROS_LIB_DIR}
)
target_link_libraries(pet_arduino_nano
  INTERFACE
    pet_arduino_core
    servo
    irremote
)

# Not nicest solution. Would like to only add the flags for code from pet_mk_iv_arduino.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")

# Generate Arduino Uno firmware
generate_arduino_firmware(arduino_uno
  SRCS src/main.cpp
  LIBS pet_arduino_uno
  BOARD uno
  PORT /dev/ArduinoUno0
)

# Generate Arduino Nano(old) firmware
generate_arduino_firmware(arduino_nano
  SRCS src/main.cpp
  LIBS pet_arduino_nano
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
