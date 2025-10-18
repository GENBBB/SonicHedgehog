import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class SlamWrapperNode(Node):
    def __init__(self):
        super().__init__('slam_toolbox_interface')

        self.get_logger().info("SLAM Toolbox Interface Node started")

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('publish_tf', True)
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/slam_toolbox/map',
            self.map_callback,
            qos
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, qos)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

    def map_callback(self, msg: OccupancyGrid):
        self.map_pub.publish(msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SlamWrapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
