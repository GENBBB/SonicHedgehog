import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import json
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Параметры
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.154)
        self.declare_parameter('update_rate', 10.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        self.ser.flush()

        self.odom_data = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.last_odl = None
        self.last_odr = None

        self.publisher = self.create_publisher(Odometry, 'odom', 10)

        self.create_timer(1 / self.update_rate, self.update_and_publish_odom)

        self.send_command({"T": 131, "cmd": 1})
        self.get_logger().info(f"OdomNode запущен. Порт: {self.port}, Частота: {self.update_rate:.1f} Hz")

    def send_command(self, cmd_dict):
        msg = json.dumps(cmd_dict) + "\r\n"
        self.ser.write(msg.encode('utf-8'))

    def get_feedback(self):
        self.send_command({"T": 130})
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            try:
                data = json.loads(line)
                if data.get("T") == 1001:
                    return float(data.get("odl", 0.0)), float(data.get("odr", 0.0))
            except json.JSONDecodeError:
                pass
        return None, None

    def update_odometry(self, odl_cm, odr_cm):
        odl = odl_cm / 100.0
        odr = odr_cm / 100.0
        if self.last_odl is None or self.last_odr is None:
            self.last_odl, self.last_odr = odl, odr
            return
        dL = odl - self.last_odl
        dR = odr - self.last_odr
        self.last_odl, self.last_odr = odl, odr
        dC = (dL + dR) / 2.0
        dTheta = (dR - dL) / self.wheel_base

        self.odom_data["theta"] += dTheta
        self.odom_data["x"] += dC * math.cos(self.odom_data["theta"])
        self.odom_data["y"] += dC * math.sin(self.odom_data["theta"])

    def update_and_publish_odom(self):
        odl, odr = self.get_feedback()
        if odl is not None and odr is not None:
            self.update_odometry(odl, odr)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.odom_data["x"]
        msg.pose.pose.position.y = self.odom_data["y"]
        msg.pose.pose.position.z = 0.0
        qz = math.sin(self.odom_data["theta"] / 2.0)
        qw = math.cos(self.odom_data["theta"] / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_command({"T": 131, "cmd": 0})
        node.ser.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
