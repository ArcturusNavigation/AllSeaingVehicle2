#! /bin/bash
sudo apt update && sudo apt upgrade

# MOOS-IvP requirements
sudo apt-get install g++ cmake xterm
sudo apt-get install libfltk1.3-dev freeglut3-dev libpng-dev libjpeg-dev 
sudo apt-get install libxft-dev libxinerama-dev libtiff5-dev

# Basic ROS package requirements
sudo apt install protobuf-compiler libb64-dev ros-humble-diagnostic-updater lsb-release wget gnupg 

# Install Gazebo Garden
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-garden python3-sdformat13 ros-humble-ros-gzgarden ros-humble-xacro

# Initialize submodule
git submodule update --init --recursive src/robot_localization src/vrx

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Set up Protobuf Gateway
protoc --cpp_out=./src/protobuf_client_ros2/include/protobuf_client ./src/protobuf_client_ros2/include/protobuf_client/gateway.proto

# Build colcon workspace
colcon build --merge-install
