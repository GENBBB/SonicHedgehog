from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    proto = os.getenv("PROTO", "tcp")
    cmd_host = os.getenv("CMD_HOST", "127.0.0.1")
    cmd_port = os.getenv("CMD_PORT", "5555")
    tel_host = os.getenv("TEL_HOST", "0.0.0.0")
    tel_port = os.getenv("TEL_PORT", "5600")

    return LaunchDescription([
        Node(
            package='webots_adapter',
            executable='webots_adapter_node',
            name='webots_adapter_node',
            output='screen',
            parameters=[{
                'proto': proto,
                'cmd_host': cmd_host,
                'cmd_port': int(cmd_port),
                'tel_host': tel_host,
                'tel_port': int(tel_port)
            }]
        ),
    ])

