import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --------- аргументы ----------
    use_sim_time = LaunchConfiguration('use_sim_time')
    my_lidar = LaunchConfiguration('my_lidar')
    send_goal = LaunchConfiguration('send_goal')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_my_lidar = DeclareLaunchArgument('my_lidar', default_value=False)
    declare_send_goal = DeclareLaunchArgument('send_goal', default_value='true')
    declare_goal_x = DeclareLaunchArgument('goal_x', default_value='1.3')
    declare_goal_y = DeclareLaunchArgument('goal_y', default_value='1.3')
    declare_goal_yaw = DeclareLaunchArgument('goal_yaw', default_value='0.0')

    pkg_path = os.path.join(
        get_package_share_directory('sonic_hedgehog'), 'configs'
    )

    # --------- пути к параметрам ----------
    slam_params = os.path.join(pkg_path, 'slam_toolbox.yaml')
    lidar_params = os.path.join(pkg_path, 'lidar.yaml')
    cmd_vel_params = os.path.join(pkg_path, 'cmd_vel.yaml')
    odom_params = os.path.join(pkg_path, 'odom.yaml')
    ekf_params = os.path.join(pkg_path, 'ekf_odom.yaml')
    behavior_server_params = os.path.join(pkg_path, 'behavior_server.yaml')
    planner_server_params = os.path.join(pkg_path, 'planner_server.yaml')
    bt_navigator_params = os.path.join(pkg_path, 'bt_navigator.yaml')
    controller_server_params = os.path.join(pkg_path, 'controller_server.yaml')

    # --------- SLAM ----------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # --------- Драйверы/симуляция ----------
    ldlidar_node = Node(
      condition=IfCondition(not my_lidar),
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'laser_frame'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 90.0},  # unit is degress
        {'angle_crop_max': 270.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 12.0}   # unit is meter
      ]
    )

    base_link_to_laser_tf_node = Node(
        condition=IfCondition(not my_lidar),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['0','0','0','0','0','0','base_link','laser_frame']
    )

    lidar_node = Node(
        condition=IfCondition(my_lidar),
        package='lidar_node',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[lidar_params, {'use_sim_time': use_sim_time}]
    )

    odom_node = Node(
        package='odom_node',
        executable='odom_node',
        name='odom_node',
        output='screen',
        parameters=[odom_params, {'use_sim_time': use_sim_time}]
    )

    cmd_vel_node = Node(
        package='cmd_vel_node',
        executable='cmd_vel_node',
        name='cmd_vel_node',
        output='screen',
        parameters=[cmd_vel_params, {'use_sim_time': use_sim_time}]
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
        condition=IfCondition(not send_goal),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_server_params, {'use_sim_time': use_sim_time}]
    )

    controller = Node(
        condition=IfCondition(not send_goal),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_server_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        condition=IfCondition(not send_goal),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        condition=IfCondition(not send_goal),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[behavior_server_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        condition=IfCondition(not send_goal),
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

    # --------- Автосенд цели ----------
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
        declare_use_sim_time,
        declare_my_lidar,
        declare_send_goal,
        declare_goal_x,
        declare_goal_y,
        declare_goal_yaw,

        # drivers
        ldlidar_node, base_link_to_laser_tf_node, lidar_node,
        odom_node, cmd_vel_node,

        # slam
        slam,

        # ekf
        ekf_node,

        # nav2
        planner, controller, bt_navigator, behavior_server, lifecycle_manager,

        # goal
        send_goal_timer,
    ])
