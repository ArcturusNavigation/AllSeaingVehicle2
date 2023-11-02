#!/bin/bash

# Start ROS2 daemon for discovery  if not already started
ros2 topic list

# Send forward command
RATE=1
CMD=2
echo "Sending forward command"
ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: ${CMD}}" &
ros2 topic pub -r ${RATE} -p 20 /wamv/thrusters/right/thrust std_msgs/msg/Float64  "{data: ${CMD}}" 
