import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json
import math

class CmdVelNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.03725)
        self.declare_parameter('wheel_base', 0.154)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        self.send_command({"T": 131, "cmd": 1})

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(f"CmdVelNode запущен. Порт: {self.port}, Скорость: {self.baudrate}")

    def send_command(self, cmd_dict):
        msg = json.dumps(cmd_dict) + "\r\n"
        self.ser.write(msg.encode('utf-8'))

    def set_velocity(self, v, omega):
        vL = v - (self.wheel_base / 2.0) * omega
        vR = v + (self.wheel_base / 2.0) * omega
        rpmL = vL / (2 * math.pi * self.wheel_radius) * 60
        rpmR = vR / (2 * math.pi * self.wheel_radius) * 60
        L = int(10 * rpmL)
        R = int(10 * rpmR)
        self.send_command({"T": 1, "L": L, "R": R})

    def cmd_vel_callback(self, msg: Twist):
        self.set_velocity(msg.linear.x, msg.angular.z)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_velocity(0, 0)
        node.send_command({"T": 131, "cmd": 0})
        node.ser.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
