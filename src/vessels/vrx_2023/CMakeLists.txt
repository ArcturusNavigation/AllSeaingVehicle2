cmake_minimum_required(VERSION 3.8)
project(vrx_2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(protobuf_client_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# install the launch and params directory
install(DIRECTORY
    launch
    params
    DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
    src/waypoints.py
    src/stationkeeping.py
    src/wildlife_simple.py
    src/acoustic_tracking_sub.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
