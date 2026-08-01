cmake_minimum_required(VERSION 3.8)
project(mpc_super_visor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mpc_super_visor_node
  src/mpc_super_visor_node.cpp
)

ament_target_dependencies(mpc_super_visor_node
  rclcpp
  std_msgs
)

install(TARGETS
  mpc_super_visor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()