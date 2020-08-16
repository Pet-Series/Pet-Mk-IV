cmake_minimum_required(VERSION 3.10.2)
project(pet_mk_iv_localisation)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    rospy
    sensor_msgs
    tf2_ros
    ugl_ros
)

find_package(ugl)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wno-deprecated-copy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pet_mk_iv_localisation
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Kalman filter shared library
add_library(kalman_filter SHARED
    src/kalman_filter.cpp
)

target_include_directories(kalman_filter
  PUBLIC
    include
)

target_link_libraries(kalman_filter
  PUBLIC
    ugl::math
  PRIVATE
    project_options
    project_warnings
)

## Kalman ROS-node executable
add_executable(kalman_node
    src/kalman_node.cpp
    src/startup_utility.cpp
    src/measurement.cpp
    src/imu_measurement.cpp
)

target_include_directories(kalman_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(kalman_node
  PUBLIC
    kalman_filter
    ${catkin_LIBRARIES}
  PRIVATE
    ugl::math
    project_options
    project_warnings
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pet_mk_iv_localisation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
