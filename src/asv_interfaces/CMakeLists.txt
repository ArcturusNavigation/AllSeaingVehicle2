cmake_minimum_required(VERSION 3.8)
project(asv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ASVState.msg"
    "msg/MOOSCommand.msg"
    "msg/LabeledBoundingBox2D.msg"
    "msg/LabeledBoundingBox2DArray.msg"
    "msg/Object.msg"
    "msg/ObjectArray.msg"
    DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
