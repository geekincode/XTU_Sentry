#!/bin/bash

cd /home/rm/sentry_nav_3.0.2
source /opt/ros/humble/setup.bash
source install/setup.bash

gnome-terminal -- /bin/bash -c 'bash sh/slam.sh ; exec bash'
sleep 4
gnome-terminal -- /bin/bash -c 'bash sh/nav.sh ; exec bash'
gnome-terminal -- /bin/bash -c 'bash sh/autoaim.sh ; exec bash'

# ros2 launch rm_vision_bringup vision_bringup.launch.py
