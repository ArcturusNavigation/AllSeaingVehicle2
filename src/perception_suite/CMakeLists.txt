cmake_minimum_required(VERSION 3.8)
project(perception_suite)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(asv_interfaces REQUIRED)

# Node: pcd_transformer
#add_executable(pcd_transformer
#  src/pcd_transformer.cpp)
#ament_target_dependencies(pcd_transformer
#  rclcpp
#  sensor_msgs
#  geometry_msgs
#  tf2_ros
#  tf2_sensor_msgs)
#
#install(TARGETS
#  pcd_transformer
#  DESTINATION lib/${PROJECT_NAME})

# install the launch and params directory
install(DIRECTORY
    #launch
  models
  DESTINATION share/${PROJECT_NAME}/)
  
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/yolov5_detector.py
  src/topic_to_image.py
  #src/pointcloud_sub.py
  src/buoy_detector.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
