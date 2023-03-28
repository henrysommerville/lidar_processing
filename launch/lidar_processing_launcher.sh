#!/bin/bash

# Check for valid bag file argument
if [ $# -ne 1 ]; then
    echo "Usage: $0 <path_to_bag_file>"
    exit 1
fi

# Check if bag file exists
if [ ! -f "$1" ]; then
    echo "Error: Bag file not found at $1"
    exit 1
fi

# Check if bag file is valid
rosbag info "$1"
if [ $? -ne 0 ]; then
    echo "Error: Invalid bag file at $1"
    exit 1
fi

# Launch lidar_processing package with specified bag file
roslaunch lidar_processing lidar_processing.launch bag_file:="$1"