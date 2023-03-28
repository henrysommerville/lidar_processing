# Lidar Processing Package

This package is designed to process lidar data collected by a mobile robot, specifically to differentiate between ground and non-ground points.

# Background

This package is designed to process lidar data collected by a mobile robot, specifically to differentiate between ground and non-ground points. The data is then visualized using rviz.

# Installation
1. Install ROS and dependencies using the following command:

    '''
    wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
    '''

2. Create a Catkin workspace using the following commands:


'''
    mkdir ~/catkin_ws/src
    cd ~/catkin_ws/src

    catkin_init_workspace

    catkin_make

    source devel/setup.bash
'''

    

3. Clone this repository into your Catkin workspace:

    '''
    cd ~/catkin_ws/src
    git clone https://github.com/<your-username>/lidar_processing.git
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
    ./lidar_processing_launcher.sh "/path/to/bag_file.bag"
    '''

