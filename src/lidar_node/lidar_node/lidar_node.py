import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import numpy as np
import math
from threading import Lock

from .lidar_parser import CalcLidarData
from .lidar_filter import filter_lidar_points, binarize_by_degree
from .lidar_stabilizer import MedianStabilizer

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.declare_parameter('com_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('conf_threshold', 40)
        self.declare_parameter('packets_per_frame', 40)
        self.declare_parameter('publish_rate', 10.0)  # Hz (запасной таймер)
        self.declare_parameter('n_bins', 360)  # число бинов для бинаризации

        self.com_port = self.get_parameter('com_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().integer_value
        self.packets_per_frame = self.get_parameter('packets_per_frame').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.n_bins = self.get_parameter('n_bins').get_parameter_value().integer_value

        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.ser = serial.Serial(port=self.com_port, baudrate=self.baudrate, timeout=0.01,
                                 bytesize=8, parity='N', stopbits=1)
        self.stabilizer = MedianStabilizer(n_bins=self.n_bins, spatial_kernel=3)

        self.angles_buf = []
        self.dist_buf = []
        self.conf_buf = []
        self.packet_count = 0
        self.lock = Lock()

        self.msg = None
        self.create_timer(1 / self.publish_rate, self.send_msg)

        self.create_timer(0.001, self.read_packets)

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = "base_link"
        static_t.child_frame_id = "laser_frame"
        static_t.transform.translation.x = 0.0
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0.0, 0.0 , 0.0)

        static_t.transform.rotation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])
        self.static_tf_broadcaster.sendTransform(static_t)
    
    def send_msg(self):
        if self.msg is None:
            return
        self.publisher.publish(self.msg)

    def read_packets(self):
        while self.ser.in_waiting:
            packet = self.read_lidar_packet()
            if packet is None:
                break

            lidar_data = CalcLidarData(packet)
            filtered = filter_lidar_points(lidar_data, self.conf_threshold)

            with self.lock:
                self.angles_buf.extend(filtered["Degree_angle"])
                self.dist_buf.extend(filtered["Distance"])
                self.conf_buf.extend(filtered["Confidence"])
                self.packet_count += 1

                if self.packet_count >= self.packets_per_frame:
                    self.publish_laserscan_locked()
                    self.angles_buf.clear()
                    self.dist_buf.clear()
                    self.conf_buf.clear()
                    self.packet_count = 0

    def publish_laserscan_locked(self):
        dist_bins, _ = binarize_by_degree(self.angles_buf, self.dist_buf, self.conf_buf, n_bins=self.n_bins)
        stable_bins = self.stabilizer.update(dist_bins)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"

        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / self.n_bins
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.publish_rate
        msg.range_min = 0.02
        msg.range_max = 8.0
        msg.ranges = [float(r) if not np.isnan(r) else float('inf') for r in stable_bins]

        self.msg = msg

    def read_lidar_packet(self):
        tmp_str = ""
        flag_54 = False
        while True:
            b = self.ser.read()
            if not b:
                return None

            val = int.from_bytes(b, 'big')

            if val == 0x54:
                tmp_str += b.hex() + " "
                flag_54 = True
            elif val == 0x2C and flag_54:
                tmp_str += b.hex()
                if len(tmp_str[0:-5].replace(" ", "")) == 90:
                    return tmp_str[0:-5]
                else:
                    tmp_str = ""
                    flag_54 = False
            else:
                tmp_str += b.hex() + " "
                flag_54 = False

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
