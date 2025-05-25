#!/bin/bash

# Generate a filename with the current date and time
filename="./maps/map_$(date +"%Y%m%d_%H%M%S")"

# Run the map_saver_cli command with the dynamic filename
ros2 run nav2_map_server map_saver_cli -t /projected_map -f "$filename" --fmt pgm

# ros2 service call /map_save std_srvs/srv/Trigger
