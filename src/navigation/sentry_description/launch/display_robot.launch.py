import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取默认路径
    pkg_path = get_package_share_directory('sentry_description')
    slam_pkg_path = get_package_share_directory('sentry_slam')
    default_model_path = pkg_path + '/urdf/fishbot/fishbot.urdf.xacro'
    urdf_rviz_config_path = pkg_path + '/config/rviz/display_model.rviz'             # 机器人urdf的 RViz 配置文件
    slam_rviz_config_path = slam_pkg_path + '/config/rviz/slam.rviz'             # 机器人slam的 RViz 配置文件


    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # 加载并激活 fishbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='log'
    )

    # 加载并激活 fishbot_effort_controller 控制器
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
        output='log')
    
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output='log')
    # RViz 节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', slam_rviz_config_path]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node,

        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=robot_state_publisher_node,
                on_exit=[load_joint_state_controller],)
            ),
        # 事件动作，load_fishbot_diff_drive_controller
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_fishbot_diff_drive_controller],)
            ),
    ])
