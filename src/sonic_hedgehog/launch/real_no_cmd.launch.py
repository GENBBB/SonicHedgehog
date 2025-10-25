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
    slam_params = os.path.join(pkg_path, 'slam_toolbox.yaml')
    lidar_params = os.path.join(pkg_path, 'lidar.yaml')
    cmd_vel_params = os.path.join(pkg_path, 'cmd_vel.yaml')
    odom_params = os.path.join(pkg_path, 'odom.yaml')
    
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
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
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

    lidar_node = Node(
        package='lidar_node',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[lidar_params, {'use_sim_time': use_sim_time}]
    )

    cmd_vel_node = Node(
        package='cmd_vel_node',
        executable='cmd_vel_node',
        name='cmd_vel_node',
        output='screen',
        parameters=[cmd_vel_params, {'use_sim_time': use_sim_time}]
    )

    odom_node = Node(
        package='odom_node',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[odom_params, {'use_sim_time': use_sim_time}]
    )


    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        slam_toolbox_node,
        configure_event,
        activate_event,
        lidar_node,
        cmd_vel_node,
        odom_node,
    ])
