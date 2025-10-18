from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox_interface',
            executable='slam_wrapper_node',
            name='slam_toolbox_interface',
            output='screen',
            parameters=['../configs/slam_toolbox.yaml']
        ),
    ])
