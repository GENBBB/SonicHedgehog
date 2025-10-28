from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition  # <-- важно

def generate_launch_description():
    # --------- аргументы ----------
    use_sim      = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf      = LaunchConfiguration('use_ekf')
    send_goal    = LaunchConfiguration('send_goal')
    goal_x       = LaunchConfiguration('goal_x')
    goal_y       = LaunchConfiguration('goal_y')
    goal_yaw     = LaunchConfiguration('goal_yaw')

    declare_use_sim      = DeclareLaunchArgument('use_sim',      default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_use_ekf      = DeclareLaunchArgument('use_ekf',      default_value='true')
    declare_send_goal    = DeclareLaunchArgument('send_goal',    default_value='true')
    declare_goal_x       = DeclareLaunchArgument('goal_x',       default_value='1.3')
    declare_goal_y       = DeclareLaunchArgument('goal_y',       default_value='1.3')
    declare_goal_yaw     = DeclareLaunchArgument('goal_yaw',     default_value='0.0')

    pkg_share = FindPackageShare('sonic_hedgehog')

    # --------- пути к параметрам ----------
    slam_params       = PathJoinSubstitution([pkg_share, 'configs', 'slam_toolbox.yaml'])
    planner_params    = PathJoinSubstitution([pkg_share, 'configs', 'planner_server.yaml'])
    controller_params = PathJoinSubstitution([pkg_share, 'configs', 'controller_server.yaml'])
    bt_params         = PathJoinSubstitution([pkg_share, 'configs', 'bt_navigator.yaml'])
    behavior_params   = PathJoinSubstitution([pkg_share, 'configs', 'behavior_server.yaml'])
    rviz_config       = PathJoinSubstitution([pkg_share, 'configs', 'rviz_config.rviz'])
    ekf_params        = PathJoinSubstitution([pkg_share, 'configs', 'ekf_odom.yaml'])
    lidar_params      = PathJoinSubstitution([pkg_share, 'configs', 'lidar.yaml'])
    odom_params       = PathJoinSubstitution([pkg_share, 'configs', 'odom.yaml'])
    cmdvel_params     = PathJoinSubstitution([pkg_share, 'configs', 'cmd_vel.yaml'])

    # --------- SLAM ----------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # --------- Драйверы/симуляция ----------
    webots_adapter = Node(
        condition=IfCondition(use_sim),
        package='webots_adapter',
        executable='webots_adapter_node',
        name='webots_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    lidar_node = Node(
        condition=UnlessCondition(use_sim),   # <-- вместо "$({not} use_sim)"
        package='lidar_node',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[lidar_params, {'use_sim_time': use_sim_time}]
    )

    odom_node = Node(
        condition=UnlessCondition(use_sim),
        package='odom_node',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[odom_params, {'use_sim_time': use_sim_time}]
    )

    cmd_vel_node = Node(
        condition=UnlessCondition(use_sim),
        package='cmd_vel_node',
        executable='cmd_vel_node',
        name='cmd_vel_node',
        output='screen',
        parameters=[cmdvel_params, {'use_sim_time': use_sim_time}]
    )

    # --------- EKF (опционально) ----------
    ekf_node = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}]
    )

    # --------- Nav2 ----------
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params, {'use_sim_time': use_sim_time}]
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    # --------- RViz ----------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --------- Автосенд цели ----------
    # Replace send_goal_cmd block with:
    send_goal_cmd = ExecuteProcess(
        condition=IfCondition(send_goal),
        shell=True,
        cmd=[
            'ros2', 'action', 'send_goal', '/navigate_to_pose', 'nav2_msgs/action/NavigateToPose',
            "{pose: { header: { frame_id: 'map' }, pose: { position: {x: ",
            goal_x,
            ", y: ",
            goal_y,
            ", z: 0.0}, orientation: {z: 0.0, w: 1.0} } } }"
        ],
        output='screen'
    )
    send_goal_timer = TimerAction(period=10.0, actions=[send_goal_cmd])

    return LaunchDescription([
        declare_use_sim,
        declare_use_sim_time,
        declare_use_ekf,
        declare_send_goal,
        declare_goal_x,
        declare_goal_y,
        declare_goal_yaw,

        # drivers / sim
        webots_adapter,
        lidar_node, odom_node, cmd_vel_node,

        # slam
        slam,

        # ekf
        ekf_node,

        # nav2
        planner, controller, bt_navigator, behavior_server, lifecycle_manager,

        # rviz
        rviz,

        # goal
        send_goal_timer,
    ])
