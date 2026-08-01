cmake_minimum_required(VERSION 3.8)
project(modelv2_mpc_limited_ros2 C CXX)

# C standard for the generated Simulink code
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++ standard for the ROS 2 node
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Warnings for code we control
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ============================================================
# Generated Simulink / Embedded Coder model
# These files are compiled as C, not C++.
# ============================================================
add_library(modelv2_gen STATIC
  src/gen/modelv2.c
  src/gen/modelv2_data.c
)

target_include_directories(modelv2_gen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link math library needed by generated C code
target_link_libraries(modelv2_gen PUBLIC m)

# Silence warnings inside auto-generated Simulink code
target_compile_options(modelv2_gen PRIVATE -w)

# ============================================================
# ROS 2 MPC wrapper node
# ============================================================
add_executable(mpc_controller_node
  src/mpc_controller_node.cpp
)

target_include_directories(mpc_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Important:
# Do NOT write PRIVATE here.
# ament_target_dependencies() uses the plain CMake linking signature.
target_link_libraries(mpc_controller_node
  modelv2_gen
)

ament_target_dependencies(mpc_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# ============================================================
# Install
# ============================================================
install(TARGETS
  mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()