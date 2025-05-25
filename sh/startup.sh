#!/bin/bash

cd /home/rm/sentry_nav_3.0.3
source /opt/ros/humble/setup.bash
source install/setup.bash

password="123456"
echo $password | sudo -S chmod 777 /dev/ttyACM0
echo "$password" | sudo -S chmod 777 /dev/ttyCH341USB0

gnome-terminal -- /bin/bash -c 'rqt ; exec bash'
gnome-terminal -- /bin/bash -c './sh/foxglove_bridge.sh ; exec bash'
sleep 3
# ros2 launch sentry_startup sentry_startup.launch.py 
gnome-terminal -- /bin/bash -c '
	source /opt/ros/humble/setup.bash;
	export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH;
	ros2 launch sentry_startup sentry_startup.launch.py ; 
	exec bash'

gnome-terminal -- /bin/bash -c '
	source /opt/ros/humble/setup.bash;
	ros2 run sentry_api judge ; 
	exec bash'
