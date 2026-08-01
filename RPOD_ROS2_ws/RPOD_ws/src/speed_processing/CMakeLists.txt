cmake_minimum_required(VERSION 3.8)
project(speed_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(speed_processing
  src/speed_processing.cpp
)

ament_target_dependencies(
  speed_processing
  rclcpp
  std_msgs
)

install(TARGETS
  speed_processing
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
