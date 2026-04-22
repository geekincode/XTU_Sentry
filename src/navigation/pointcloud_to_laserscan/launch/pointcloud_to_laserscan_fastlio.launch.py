"""
Launch file for pointcloud_to_laserscan node to convert FAST-LIO point cloud to laser scan.
This integrates with FAST-LIO's cloud_registered topic for 2D SLAM and navigation.

Usage:
    ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_fastlio.launch.py
    
    Or with parameters:
    ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_fastlio.launch.py \
        min_height:=-0.1 max_height:=0.1 range_min:=0.5 range_max:=30.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    min_height = DeclareLaunchArgument(
        'min_height',
        default_value='-0.15',
        description='Minimum height for laser scan extraction from point cloud'
    )
    
    max_height = DeclareLaunchArgument(
        'max_height',
        default_value='0.15',
        description='Maximum height for laser scan extraction from point cloud'
    )
    
    angle_min = DeclareLaunchArgument(
        'angle_min',
        default_value='-1.5708',  # -M_PI/2
        description='Minimum angle of the laser scan (radians)'
    )
    
    angle_max = DeclareLaunchArgument(
        'angle_max',
        default_value='1.5708',  # M_PI/2
        description='Maximum angle of the laser scan (radians)'
    )
    
    angle_increment = DeclareLaunchArgument(
        'angle_increment',
        default_value='0.0087',  # M_PI/360.0 (0.5 degree increment)
        description='Angular resolution of the laser scan (radians)'
    )
    
    range_min = DeclareLaunchArgument(
        'range_min',
        default_value='0.5',
        description='Minimum range of laser scan (meters)'
    )
    
    range_max = DeclareLaunchArgument(
        'range_max',
        default_value='30.0',
        description='Maximum range of laser scan (meters)'
    )
    
    target_frame = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link',
        description='Target frame for coordinate transformation'
    )
    
    transform_tolerance = DeclareLaunchArgument(
        'transform_tolerance',
        default_value='0.01',
        description='Transform timeout (seconds)'
    )
    
    use_inf = DeclareLaunchArgument(
        'use_inf',
        default_value='true',
        description='Use infinity for out-of-range points'
    )

    # Point cloud to laser scan conversion node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),  # Subscribe to FAST-LIO cloud_registered
            ('scan', '/scan')                    # Publish laser scan
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('target_frame'),
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'angle_increment': LaunchConfiguration('angle_increment'),
            'scan_time': 0.1,
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'use_inf': LaunchConfiguration('use_inf'),
            'inf_epsilon': 1.0
        }],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(min_height)
    ld.add_action(max_height)
    ld.add_action(angle_min)
    ld.add_action(angle_max)
    ld.add_action(angle_increment)
    ld.add_action(range_min)
    ld.add_action(range_max)
    ld.add_action(target_frame)
    ld.add_action(transform_tolerance)
    ld.add_action(use_inf)
    
    # Add nodes
    ld.add_action(pointcloud_to_laserscan_node)
    
    return ld
