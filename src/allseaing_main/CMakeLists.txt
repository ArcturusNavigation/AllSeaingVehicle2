cmake_minimum_required(VERSION 3.8)
project(allseaing_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(asv_interfaces REQUIRED)

# Node: state_reporter
add_executable(state_reporter
  src/state_reporter.cpp)
ament_target_dependencies(state_reporter
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  asv_interfaces)

install(TARGETS
  state_reporter
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
