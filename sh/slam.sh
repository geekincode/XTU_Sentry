#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash

password="123456"
echo $password | sudo -S chmod 777 /dev/ttyACM0

# gnome-terminal -- /bin/bash -c 'ros2 launch livox_ros_driver2 msg_MID360_launch.py ; exec bash'
# gnome-terminal -- /bin/bash -c 'ros2 launch fast_lio mapping.launch.py ; exec bash'
# gnome-terminal -- /bin/bash -c 'ros2 launch octomap_server2 octomap_server_launch.py ; exec bash'

# gnome-terminal -- /bin/bash -c 'ros2 launch ego_planner rviz.launch.py ; exec bash'
ros2 launch fast_lio slam_main.launch.py

# gnome-terminal -- /bin/bash -c 'ros2 launch ego_planner run_in_sim.launch.py ; exec bash'
