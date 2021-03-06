cmake_minimum_required(VERSION 3.10.2)
project(pet_mk_iv_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  ugl_ros
)

find_package(Ceres REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

find_package(ugl REQUIRED)

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
#  LIBRARIES pet_mk_iv_control
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_library(mpc SHARED
  src/mpc.cpp
  src/kinematic_model.cpp
  src/parameterization2d.cpp
  src/pose2d.cpp
  src/residuals.cpp
  src/rotation2d.cpp
  src/setpoint.cpp
  src/trajectory.cpp
)

target_include_directories(mpc
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(mpc
  PUBLIC
    Eigen3::Eigen
    Ceres::ceres
    ugl::math
    ${catkin_LIBRARIES}
  PRIVATE
    project_options
    project_warnings
)

add_library(mpc_ros
  src/mpc_node.cpp
  src/mpc_ros.cpp
)
target_include_directories(mpc_ros
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mpc_ros
  PUBLIC
    mpc
    ${catkin_LIBRARIES}
  PRIVATE
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

add_subdirectory(test)
