import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取与拼接默认路径
    sentry_navigation_dir = get_package_share_directory('sentry_navigation')
    slam_pkg_share = get_package_share_directory('sentry_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 创建 Launch 配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(slam_pkg_share, 'maps', 'map_20250505_143854.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(sentry_navigation_dir, 'config', 'nav2_params.yaml'))

    # 创建导航启动描述
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    # 创建 SLAM 启动描述
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg_share, 'launch', 'cartographer.launch.py')
        )
    )

    # 创建静态变换节点
    static_transform_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0.0', '0', '0', '0', '0', '0', 'odom', 'base_link']  # XYZ RPY
    )

    # 启动 LiDAR、IMU、Fast LiO 和 Octomap Server 进程
    launch_livox = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
        output='screen'
    )

    launch_dm_imu = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'dm_imu', 'run_without_rviz.launch.py'],
        output='screen'
    )

    launch_fast_lio2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'fast_lio', 'mapping.launch.py'],
        output='screen'
    )

    launch_octomap_server2 = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'octomap_server', 'octomap_mapping.launch.xml'],
        output='screen'
    )

    launch_autoaim = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'rm_vision_bringup', 'vision_bringup.launch.py'],
        output='screen'
    )

    # 创建一个动作组，用于按顺序启动节点和进程
    action_group = launch.actions.GroupAction([
        launch.actions.TimerAction(period=1.0, actions=[launch_livox]),
        launch.actions.TimerAction(period=2.0, actions=[launch_dm_imu]),
        launch.actions.TimerAction(period=4.0, actions=[launch_fast_lio2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_octomap_server2]),
        launch.actions.TimerAction(period=5.0, actions=[launch_autoaim]),
        launch.actions.TimerAction(period=6.0, actions=[navigation_launch]),
        launch.actions.TimerAction(period=7.0, actions=[slam_launch]),
    ])

    # 创建 RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log'
    )

    # 声明 Launch 参数
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_path, description='Full path to map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path, description='Full path to param file to load'),
    ]

    # 返回 LaunchDescription
    return launch.LaunchDescription([
        *declared_arguments,
        static_transform_node,
        rviz_node,
        action_group
    ])