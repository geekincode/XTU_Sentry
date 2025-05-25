import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    sentry_navigation_dir = get_package_share_directory(
        'sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    # slam_map_path = os.path.join(
    #     sentry_navigation_dir, 'maps', 'map_slam.yaml')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(slam_pkg_share, 'maps', 'map_slam.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))
    print(os.path.join(slam_pkg_share, 'maps', 'map_slam.yaml'))

    sentry_discription_path = get_package_share_directory('sentry_description')
        # 定位到功能包的地址
    # pkg_share = get_package_share_directory('sentry_serial')
    # ekf_param_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # print(ekf_param_path)

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log'),

        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_localization_node',
        #     arguments=['-d', ekf_param_path],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='log'),

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link'
            ],
            output='screen'
        ),
    ])