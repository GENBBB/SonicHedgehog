from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Пути к конфигам ---
    pkg_path = os.path.join(
        get_package_share_directory('sonic_hedgehog'), 'configs'
    )
    slam_params_path = os.path.join(pkg_path, 'slam_toolbox.yaml')
    rviz_config_path = os.path.join(pkg_path, "rviz_config.rviz")

    rviz_config_path = os.path.join(get_package_share_directory('sonic_hedgehog'), 'rviz', 'rviz_config.rviz')

    # --- Переменные окружения ---
    cmd_host = str(os.getenv("CMD_HOST", "127.0.0.1"))
    cmd_port = int(os.getenv("CMD_PORT", "5555"))
    tel_host = str(os.getenv("TEL_HOST", "0.0.0.0"))
    tel_port = int(os.getenv("TEL_PORT", "5600"))
    proto = str(os.getenv("PROTO", "tcp"))

    # --- Аргументы ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation Webots time'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically configure and activate slam_toolbox'
    )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_path, {'use_sim_time': use_sim_time}]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(autostart)
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[Lifecycle] Activating slam_toolbox...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(autostart)
    )

    webots_adapter_node = Node(
        package='webots_adapter',
        executable='webots_adapter_node',
        name='webots_adapter_node',
        output='screen',
        parameters=[{
            'proto': proto,
            'cmd_host': cmd_host,
            'cmd_port': cmd_port,
            'tel_host': tel_host,
            'tel_port': tel_port
        }]
    )

    visualization_node = Node(
        package='visualization',
        executable='visualization_node',
        name='visualization',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        slam_toolbox_node,
        configure_event,
        activate_event,
        webots_adapter_node,
        visualization_node,
        rviz_node,
    ])
