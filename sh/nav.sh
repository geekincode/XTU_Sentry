#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

password="123456"
echo $password | sudo -S chmod 777 /dev/ttyCH341USB0

ros2 launch sentry_navigation nav_aim.launch.py

