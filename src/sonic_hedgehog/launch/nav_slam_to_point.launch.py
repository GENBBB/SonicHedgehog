import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, EmitEvent, LogInfo, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LifecycleNode
from launch.conditions import IfCondition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition

def generate_launch_description():
    # --------- аргументы ----------
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_send_goal = DeclareLaunchArgument('send_goal', default_value='true')

    use_sim_time = LaunchConfiguration('use_sim_time')
    send_goal = LaunchConfiguration('send_goal')
    goal_x = 0.0
    goal_y = 2.6
    goal_yaw = 0.0

    pkg_path = os.path.join(
        get_package_share_directory('sonic_hedgehog'), 'configs'
    )

    # --------- пути к параметрам ----------
    slam_params = os.path.join(pkg_path, 'slam_toolbox.yaml')
    cmd_vel_params = os.path.join(pkg_path, 'cmd_vel.yaml')
    odom_params = os.path.join(pkg_path, 'odom.yaml')
    ekf_params = os.path.join(pkg_path, 'ekf_odom.yaml')
    behavior_params = os.path.join(pkg_path, 'behavior_server.yaml')
    planner_params = os.path.join(pkg_path, 'planner_server.yaml')
    bt_params = os.path.join(pkg_path, 'bt_navigator.yaml')
    controller_params = os.path.join(pkg_path, 'controller_server.yaml')

    ldlidar_params = os.path.join(pkg_path, 'ld_lidar.yaml')
    driver_params = os.path.join(pkg_path, 'cobraflex_driver.yaml')
    urdf_file_name = 'ldlidar_descr.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('ldlidar_node'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # --------- SLAM ----------
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[Lifecycle] Activating slam_toolbox...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
    )

    # --------- Lidar ----------

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ldlidar_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    container_name_val = 'ldlidar_container'
    distro = os.environ['ROS_DISTRO']
    if distro == 'foxy':
        container_exec='component_container'
    else:
        container_exec='component_container_isolated'
    
    ldlidar_container = ComposableNodeContainer(
            name=container_name_val,
            namespace='',
            package='rclcpp_components',
            executable=container_exec,
            composable_node_descriptions=[
                ComposableNode(
                    package='ldlidar_component',
                    plugin='ldlidar::LdLidarComponent',
                    name='ld_lidar',
                    parameters=[ldlidar_params],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )

    # odom_node = Node(
    #     package='odom_node',
    #     executable='odom_node',
    #     name='odom_node',
    #     output='screen',
    #     parameters=[odom_params, {'use_sim_time': use_sim_time}]
    # )

    # cmd_vel_node = Node(
    #     package='cmd_vel_node',
    #     executable='cmd_vel_node',
    #     name='cmd_vel_node',
    #     output='screen',
    #     parameters=[cmd_vel_params, {'use_sim_time': use_sim_time}]
    # )

    cobraflex_driver = Node(
        package='cobraflex_driver',
        executable='cobraflex_driver',
        name='cobraflex_driver',
        output='screen',
        parameters=[driver_params, {'use_sim_time': use_sim_time}]
    )

    # --------- EKF ----------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}]
    )

    # --------- Nav2 ----------
    planner = Node(
        condition=IfCondition(send_goal),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params, {'use_sim_time': use_sim_time}]
    )

    controller = Node(
        condition=IfCondition(send_goal),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        condition=IfCondition(send_goal),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        condition=IfCondition(send_goal),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        condition=IfCondition(send_goal),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'ld_lidar',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    # --------- Автосенд цели ----------
    # Replace send_goal_cmd block with:
    goal_str = (
        '"{pose: {header: {frame_id: \'map\'}, pose: {position: {x: ' 
        + str(goal_x) 
        + ', y: ' + str(goal_y) 
        + ', z: 0.0 }, orientation: { z: 0.0, w: 1.0}}}}"'
    )
    send_goal_cmd = ExecuteProcess(
        condition=IfCondition(send_goal),
        shell=True,
        cmd=[
            'ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose',
            goal_str
        ],
        output='screen'
    )
    send_goal_timer = TimerAction(period=10.0, actions=[send_goal_cmd])

    return LaunchDescription([
        declare_use_sim_time,
        declare_send_goal,

        # drivers
        rsp_node, ldlidar_container,
        # odom_node, cmd_vel_node,
        cobraflex_driver,
        # slam
        slam_node,

        configure_event, activate_event,

        # ekf
        ekf_node,

        # nav2
        planner, controller, bt_navigator, behavior_server, lifecycle_manager,

        # goal
        send_goal_timer,
    ])
