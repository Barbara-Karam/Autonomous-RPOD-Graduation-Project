cmake_minimum_required(VERSION 3.8)
project(statespace_chaser_mpc_ros2 C CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Generated Simulink/Embedded Coder controller, compiled as plain C ---
add_library(statespace_chaser_gen STATIC
  src/gen/statespace_chaser_test0x2810x29.c
  src/gen/statespace_chaser_test0x2810x29_data.c
)
target_include_directories(statespace_chaser_gen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(statespace_chaser_gen PUBLIC m)

# --- ROS 2 node that wraps it ---
add_executable(mpc_controller_node src/mpc_controller_node.cpp)
target_include_directories(mpc_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mpc_controller_node statespace_chaser_gen)
ament_target_dependencies(mpc_controller_node
  rclcpp geometry_msgs std_msgs tf2 tf2_geometry_msgs)

install(TARGETS mpc_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()