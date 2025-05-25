import os.path
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():

    launch_livox = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2','msg_MID360_launch.py'], 
        output='screen')
    
    launch_dm_imu = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'dm_imu','run_without_rviz.launch.py'], 
        output='screen')

    launch_fast_lio2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio','mapping.launch.py'], 
        output='screen')
    
    launch_octomap_server2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'octomap_server','octomap_mapping.launch.xml'], 
        output='screen')
    
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[launch_livox]),
        launch.actions.TimerAction(period=2.0, actions=[launch_dm_imu]),
        launch.actions.TimerAction(period=4.0, actions=[launch_fast_lio2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_octomap_server2]),
    ])

    return launch.LaunchDescription([
        action_group
    ])

