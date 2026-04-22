"""
Integrated FAST-LIO + Point2LaserScan + Cartographer 2D SLAM pipeline with RViz visualization.

This launch file chains together:
1. Point cloud to laser scan conversion (/cloud_registered → /scan)
2. Cartographer 2D SLAM (/scan → /map)
3. RViz2 visualization with Cartographer config

Available point cloud to laser scan configurations:
  - fastlio_default.yaml     : Optimized default settings
  - fastlio_high_density.yaml: High density scans (for sparse data)
  - fastlio_balanced.yaml    : Balanced settings
  - fastlio_performance.yaml : Performance-optimized (embedded systems)

Usage (requires FAST-LIO running):
    # Basic usage with default config
    ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py
    
    # Use high-density point cloud conversion
    ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py \
        config_file:=fastlio_high_density.yaml
        
    # Override specific parameters
    ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py \
        use_sim_time:=true min_height:=-0.2 max_height:=0.2 range_max:=40.0
        
    # Disable RViz
    ros2 launch pointcloud_to_laserscan fastlio_cartographer_slam.launch.py \
        use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare(package='pointcloud_to_laserscan').find('pointcloud_to_laserscan')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value='fastlio_high_density.yaml',
        description='Point cloud to laser scan YAML config file. Options: fastlio_default.yaml, fastlio_high_density.yaml, fastlio_balanced.yaml, fastlio_performance.yaml'
    )
    
    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='cartographer.rviz',
        description='RViz config file name (in config directory)'
    )
    
    # Build path to YAML config file
    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('config_file')
    ])
    
    # Build path to RViz config file
    rviz_config_path = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('rviz_config')
    ])
    
    # Point cloud to laser scan node - load parameters from YAML file
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[config_file_path],
        output='screen'
    )
    
    # Cartographer node
    configuration_directory = os.path.join(pkg_share, 'config')
    configuration_basename = 'sentry_2d.lua'
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/points2', '/scan'),
            ('/imu', '/imu/data'),
            ('/odom', '/Odometry')
        ]
    )
    
    # Cartographer occupancy grid node
    cartographer_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    # RViz2 node for visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_time)
    ld.add_action(config_file)
    ld.add_action(use_rviz)
    ld.add_action(rviz_config)
    
    # Add nodes
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid)
    
    # Conditionally add RViz
    from launch.conditions import IfCondition
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        )
    )
    
    return ld
