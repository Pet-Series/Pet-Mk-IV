
add_executable(ultrasound_adapter_node
    ultrasound_adapter_node.cpp
)

target_include_directories(ultrasound_adapter_node
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(ultrasound_adapter_node
  PRIVATE
    ${catkin_LIBRARIES}
)

target_compile_features(ultrasound_adapter_node
  PRIVATE
    cxx_std_17
)

target_compile_options(ultrasound_adapter_node
  PRIVATE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wno-deprecated-copy
)
