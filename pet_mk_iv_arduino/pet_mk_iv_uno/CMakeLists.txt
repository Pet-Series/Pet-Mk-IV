cmake_minimum_required(VERSION 3.10.2)

project(pet_mk_iv_uno)

find_package(pet_mcu_common REQUIRED
  COMPONENTS
    core
    engine_module
    line_sensor_module
    ultrasound_module
    ultrasound_legacy_module
)

include(${ROSSERIAL_ARDUINO_TOOLCHAIN})
include(Platform/Arduino)

register_hardware_platform(${ARDUINO_SDK_PATH}/hardware/arduino)

# TODO: Convert to target-oriented design.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARDUINO_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ARDUINO_LINKER_FLAGS}")

function(target_copy_property TARGET_FROM TARGET_TO PROPERTY)
  get_target_property(PROPERTY_VALUE ${TARGET_TO} ${PROPERTY})
  get_target_property(ADDITIONAL_PROPERTY_VALUE ${TARGET_FROM} ${PROPERTY})
  if(NOT PROPERTY_VALUE)
    set(PROPERTY_VALUE)
  endif()
  if(ADDITIONAL_PROPERTY_VALUE)
    list(APPEND PROPERTY_VALUE ${ADDITIONAL_PROPERTY_VALUE})
    list(REMOVE_DUPLICATES PROPERTY_VALUE)
    set_property(TARGET ${TARGET_TO} PROPERTY ${PROPERTY} ${PROPERTY_VALUE})
  endif()
endfunction()

add_library(pet_mk_iv_uno_deps INTERFACE)
foreach(DEPENDENCY pet::mcu_core pet::ros_lib pet::ultrasound_module pet::ultrasound_legacy_module pet::new_ping pet::engine_module pet::line_sensor_module)
  target_copy_property(${DEPENDENCY} pet_mk_iv_uno_deps INTERFACE_SOURCES)
  target_copy_property(${DEPENDENCY} pet_mk_iv_uno_deps INTERFACE_INCLUDE_DIRECTORIES)
endforeach()

# Generate Arduino Uno firmware
generate_arduino_firmware(pet_mk_iv_uno
  SRCS src/modules.cpp
  LIBS pet_mk_iv_uno_deps
  BOARD uno
  PORT /dev/ArduinoUno0
  NO_AUTOLIBS
)

target_compile_options(pet_mk_iv_uno PRIVATE "-Os")
