import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from .sim_telemetry_node import SimTelemetry
from .sim_cmdvel_node import SimCmdVel
import transforms3d.euler as t3d_euler
import threading

class WebotsAdapterNode(Node):
    def __init__(self):
        super().__init__('webots_adapter_node')

        proto = self.get_parameter_or('proto', 'tcp')
        cmd_host = self.get_parameter_or('cmd_host', '127.0.0.1')
        cmd_port = self.get_parameter_or('cmd_port', 5555)
        tel_host = self.get_parameter_or('tel_host', '0.0.0.0')
        tel_port = self.get_parameter_or('tel_port', 5600)
        self.rate_hz = self.get_parameter_or('rate_hz', 20.0)

        self.get_logger().info(
            f"Starting Webots Adapter: CMD {cmd_host}:{cmd_port}, TEL {tel_host}:{tel_port}, PROTO {proto}"
        )

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)

        self.telemetry = SimTelemetry(self.get_logger(), tel_host, tel_port, proto)
        self.cmdvel = SimCmdVel(self.get_logger(), cmd_host, cmd_port, proto)

        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = "base_link"
        static_t.child_frame_id = "laser_frame"
        static_t.transform.translation.x = 0.1
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.2
        static_t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.static_tf_broadcaster.sendTransform(static_t)

    def cmd_callback(self, msg: Twist):
        self.cmdvel.send_cmd(msg.linear.x, msg.angular.z)

    def _telemetry_loop(self):
        r = 1.0 / self.rate_hz
        while rclpy.ok():
            data = self.telemetry.recv_tel()
            if data is not None and len(data) >= 6:
                self._publish_data(data)
            self.telemetry.sleep(r)

    def _publish_data(self, data):
        x, y, th, vel, gyro, ranges = data
        x = float(x)
        y = float(y)
        th = float(th)
        vx = float(vel[0])
        vy = float(vel[1])
        vth = float(vel[2])

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        quat = t3d_euler.euler2quat(0, 0, th, axes='sxyz')
        qx, qy, qz, qw = map(float, [quat[1], quat[2], quat[3], quat[0]])
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        if ranges:
            scan = LaserScan()
            scan.header.stamp = odom.header.stamp
            scan.header.frame_id = "laser_frame"
            scan.angle_min = -1.57 / 2
            scan.angle_max = 1.57 / 2
            scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(ranges) - 1)
            scan.range_min = 0.1
            scan.range_max = 8.0
            scan.ranges = [float(r) if r < 8.0 else float('inf') for r in ranges]
            scan.time_increment = 1.0 / (self.rate_hz * len(ranges))
            scan.scan_time = 1.0 / self.rate_hz
            self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = WebotsAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()