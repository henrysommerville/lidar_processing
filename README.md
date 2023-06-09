# Lidar Processing Package

This package is designed to process lidar data collected by a mobile robot using ROS, specifically to differentiate between ground and non-ground points.

# Background

This package was created for a mobile robot project using ROS and Python, as a skills test for Decoda. The package provides a node to process raw lidar data and output segmented point clouds containing only ground or non-ground points. The algorithm used for this segmentation is based on a variation of RANSAC algorithm.


# Installation
1. Install ROS and dependencies using the following command:

'''

    wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
    
'''

2. Create a Catkin workspace using the following commands:


'''

    mkdir -p catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    source devel/setup.bash
    
'''

    

3. Clone this repository into your Catkin workspace:

'''
    
    cd ~/catkin_ws/src
    git clone https://github.com/henrysommerville/lidar_processing
  
'''

4. Build the Catkin workspace and source:

'''
  
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
  
'''
  

# Usage

1. Navigate to the launch directory inside the package directory:

'''
  
    cd ~/catkin_ws/src/lidar_processing/launch
  
'''

2. Run the following command to launch the lidar_processing process, passing the bag file as a command line argument:

'''
  
    ./lidar_processing_launcher.sh "</path/to/bag_file.bag>"
  
'''
