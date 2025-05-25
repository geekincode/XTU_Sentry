import os
import launch
import launch_ros
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    sentry_navigation_dir = get_package_share_directory(
        'sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(slam_pkg_share, 'maps', 'map_20250505_143854.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))
    print(os.path.join(slam_pkg_share, 'maps', 'map_slam.yaml'))
        # 定位到功能包的地址
    # pkg_share = get_package_share_directory('sentry_serial')
    # ekf_param_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # print(ekf_param_path)

    slam_launch=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("sentry_slam"),
                "launch",
                "cartographer.launch.py"
            )
        )
    )

    rm_serial_driver_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("rm_serial_driver"),
                "launch",
                "serial_driver.launch.py"
            )
        )
    )

    # nav_to_pose_node = launch_ros.actions.Node(
    #         package='sentry_application',
    #         executable='nav_to_pose',
    #         name='nav_to_pose',
    #         output='log')
    
    # pose_cmd= launch.actions.ExecuteProcess(cmd=['ros2', 'run', 'sentry_application', 'nav_to_pose'],
    #     output='log')
    
    # poses_cmd= launch.actions.ExecuteProcess(cmd=['ros2', 'run', 'sentry_application', 'waypoint_follower'],
    #     output='log')

    # ekf_node = launch_ros.actions.Node(
    #         package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         arguments=['-d', ekf_param_path],
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output='log')
    
    # ekf_cmd= launch.actions.ExecuteProcess(cmd=['ros2', 'run', 'robot_localization', 'ekf_node',
    #          '--ros-args', '--params-file',
    #          '/home/rm/Desktop/sentry_nav_V2.3.3/install/sentry_serial/share/sentry_serial/config/ekf.yaml'],
    #     output='log')
    
    # ekf = launch.actions.TimerAction(period=10.0, actions=[ekf_node])

    static_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['-0.0', '0', '0', '0', '0', '0', 'odom', 'base_link']       # XYZ RPY
    )

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

        slam_launch,
        static_transform_node,
        
        # launch_autoaim,
        # pose_cmd,
        # poses_cmd,
        # nav_to_pose_node,

        # ekf_node,
        # ekf_cmd,
        
    #    rm_serial_driver_launch,
        # laser_launch,

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log'),

    ])

