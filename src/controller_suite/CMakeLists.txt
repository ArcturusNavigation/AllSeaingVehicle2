cmake_minimum_required(VERSION 3.8)
project(controller_suite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asv_interfaces REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
    launch
    params
    resource
    DESTINATION share/${PROJECT_NAME}/)

# Install Python executables
install(PROGRAMS
  src/simple_controller.py
  src/xdrive_controller.py
  src/sample_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
