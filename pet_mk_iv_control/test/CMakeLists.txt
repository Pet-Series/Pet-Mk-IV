add_executable(prototype_test
  src/prototype_test.cpp
)

target_include_directories(prototype_test
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(prototype_test
  PUBLIC
    mpc
    mpc_ros
    ugl::math
    ${catkin_LIBRARIES}
  PRIVATE
    project_options
    project_warnings
)
