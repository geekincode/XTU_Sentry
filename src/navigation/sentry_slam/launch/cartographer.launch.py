import os
import launch_ros
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 文件包含相关----------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='sentry_slam').find('sentry_slam')
    default_rviz_config_path = pkg_share + '/config/rviz/slam.rviz'
    
    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='sentry_3d.lua')     # simulation_2d.lua

    # pkg_share = get_package_share_directory('sentry_serial')
    # ekf_param_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    # print(ekf_param_path)

    # ekf_node = launch_ros.actions.Node(
    #         package='robot_localization',
    #         executable='ekf_node',
    #         name='ekf_filter_node',
    #         arguments=['-d', ekf_param_path],
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output='log')

    
    #=====================声明四个节点，laser_node/cartographer/occupancy_grid_node/rviz_node=================================
    # 启动A2M8雷达
    # laser_launch=IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         launch_file_path=os.path.join(
    #             get_package_share_directory("sllidar_ros2"),
    #             "launch",
    #             "sllidar_a2m8_launch.py"
    #         )
    #     )
    # )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings=[
        # 格式：('原始话题', '新话题')
        ('/points2', '/cloud_registered'),          # 点云数据
        ('/imu', '/imu/data'),                # IMU数据（如使用）
        ('/odom', '/Odometry')]              # 里程计数据（如使用外部里程计）
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    # 启动rviz
    display_launch=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(
                get_package_share_directory("sentry_description"),
                "launch",
                "display_robot.launch.py"
            )
        )
    )

    # 启动串口
    # serial_driver_launch=IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         launch_file_path=os.path.join(
    #             get_package_share_directory("sentry_serial"),
    #             "launch",
    #             "serial_driver.launch.py"
    #         )
    #     )
    # )

    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    # ld.add_action(serial_driver_launch)
    # ld.add_action(laser_launch)
    # ld.add_action(ekf_node)
    # ld.add_action(display_launch)
    ld.add_action(cartographer_node)
    # ld.add_action(cartographer_occupancy_grid_node)       #!problem

    return ld
