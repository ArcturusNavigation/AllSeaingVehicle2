#! /bin/bash
sudo apt update && sudo apt upgrade

# MOOS-IvP requirements
sudo apt-get install -y g++ cmake xterm
sudo apt-get install -y libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev 
sudo apt-get install -y libxft-dev libxinerama-dev libtiff5-dev libprotobuf-dev

# Basic ROS package requirements
sudo apt install -y protobuf-compiler libb64-dev ros-foxy-diagnostic-updater lsb-release wget gnupg 

# Initialize submodule
git submodule update --init --recursive src/robot_localization

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy

# Set up Protobuf Gateway
protoc --cpp_out=./src/protobuf_client_ros2/include/protobuf_client ./src/protobuf_client_ros2/include/protobuf_client/gateway.proto

# Build colcon workspace
colcon build
