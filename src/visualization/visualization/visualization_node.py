import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        self.robot_pose = None
        self.prev_position = None

        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info("Visualization node started")

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def publish_markers(self):
        if self.robot_pose is None:
            return
        
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'robot'
        robot_marker.id = 0
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose = self.robot_pose
        robot_marker.scale.x = 0.2
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.1
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0

        self.marker_pub.publish(robot_marker)

        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'map'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = 'robot_direction'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position = self.robot_pose.position

        arrow_marker.pose.position = self.robot_pose.position
        arrow_marker.pose.orientation = self.robot_pose.orientation

        arrow_marker.scale.x = 0.4  # длина стрелки
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.05
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        self.marker_pub.publish(arrow_marker)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()