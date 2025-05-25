#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

password="123456"
echo "$password" | sudo -S chmod 777 /dev/ttyCH341USB0

gnome-terminal -- /bin/bash -c 'rqt ; exec bash'
gnome-terminal -- /bin/bash -c './sh/foxglove_bridge.sh ; exec bash'
ros2 launch rm_vision_bringup vision_bringup.launch.py

# read -p "是否执行 rqt(相机画面)? (y/n): " choice
# if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
#     # 打开新终端窗口并执行 foxglove_bridge.sh
#     gnome-terminal -- /bin/bash -c 'rqt ; exec bash'
#     # rqt &
#     # echo "foxglove_bridge.sh 的PID是: $!"
# else
#     echo "跳过执行 rqt"
# fi

# read -p "是否执行 foxglove_bridge(自瞄可视化桥接器)? (y/n): " choice
# if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
#     # 打开新终端窗口并执行 foxglove_bridge.sh
#     gnome-terminal -- /bin/bash -c './sh/foxglove_bridge.sh ; exec bash'
#     # echo "foxglove_bridge.sh 的PID是: $!"
# else
#     echo "跳过执行 foxglove_bridge.sh"
# fi

# # ros2 launch rm_vision_bringup vision_bringup.launch.py --log-level DEBUG
# ros2 launch rm_vision_bringup vision_bringup.launch.py

# while true; do
#     read -p "是否继续运行？(y/n) " yn
#     case $yn in
#         [Yy]* ) break;;
#         [Nn]* ) exit;;
#         * ) echo "请输入 y 或 n.";;
#     esac
# done