#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CobraFlex unified serial driver for ROS 2 (Nav2 + slam_toolbox)
- Subscribes: /cmd_vel (geometry_msgs/Twist)
- Publishes:  /odom (nav_msgs/Odometry)
- Broadcasts: odom -> base_link TF

IMPORTANT:
  * This node is the **only** owner of the serial port. Do not run any other
    node that also opens the same /dev/ttyACMx.
  * The MCU is expected to speak JSON lines over serial. By default, this node
    sends velocity commands as: {"T": 1, "L": <rpm_left>, "R": <rpm_right>}\n
    Odometry is expected as JSON lines that contain *cumulative wheel travel*
    since power‑on for the left and right wheels (e.g. centimeters):
        {"odl": <float>, "odr": <float>}
    If your MCU requires polling, set poll_odom:=True and odom_poll_template
    to the JSON you need to send (string), e.g. '{"T":7}'.

Tested with rclpy >= Humble. Adjust parameters for your hardware.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

try:
    import serial  # pyserial
    from serial import SerialException
except Exception as e:  # pragma: no cover
    serial = None
    SerialException = Exception


# ----------------------------- Helpers ------------------------------------ #

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# ------------------------------ Node -------------------------------------- #

class CobraflexDriver(Node):
    def __init__(self) -> None:
        super().__init__('cobraflex_driver')

        # ---- Parameters (declare + read) ----
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.03725)   # [m]
        self.declare_parameter('wheel_base', 0.154)       # [m] (axle distance)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('max_rpm', 150.0)
        self.declare_parameter('odom_rate', 50.0)         # [Hz]
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('cov_pose_xy', 2.0e-2)      # rough defaults
        self.declare_parameter('cov_pose_yaw', 5.0e-2)
        self.declare_parameter('cov_twist_xy', 5.0e-2)
        self.declare_parameter('cov_twist_yaw', 1.0e-1)

        # Odometry stream configuration
        self.declare_parameter('poll_odom', True)
        self.declare_parameter('odom_poll_template', '{"T": 130}')
        self.declare_parameter('left_key', 'odl')
        self.declare_parameter('right_key', 'odr')
        self.declare_parameter('units', 'cm')  # 'cm' or 'm'

        # Watchdog: stop motors if no /cmd_vel for this time (s)
        self.declare_parameter('cmd_timeout', 0.5)

        self.port: str = self.get_parameter('port').value
        self.baud: int = int(self.get_parameter('baudrate').value)
        self.R: float = float(self.get_parameter('wheel_radius').value)
        self.W: float = float(self.get_parameter('wheel_base').value)
        self.invL: bool = bool(self.get_parameter('invert_left').value)
        self.invR: bool = bool(self.get_parameter('invert_right').value)
        self.max_rpm: float = float(self.get_parameter('max_rpm').value)
        self.odom_rate: float = float(self.get_parameter('odom_rate').value)
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.odom_frame: str = self.get_parameter('odom_frame').value
        self.base_link_frame: str = self.get_parameter('base_link_frame').value
        self.poll_odom: bool = bool(self.get_parameter('poll_odom').value)
        self.odom_poll_template: str = self.get_parameter('odom_poll_template').value
        self.left_key: str = self.get_parameter('left_key').value
        self.right_key: str = self.get_parameter('right_key').value
        self.units: str = self.get_parameter('units').value
        self.cmd_timeout: float = float(self.get_parameter('cmd_timeout').value)

        self.cov_pose_xy = float(self.get_parameter('cov_pose_xy').value)
        self.cov_pose_yaw = float(self.get_parameter('cov_pose_yaw').value)
        self.cov_twist_xy = float(self.get_parameter('cov_twist_xy').value)
        self.cov_twist_yaw = float(self.get_parameter('cov_twist_yaw').value)

        # ---- Open serial ----
        if serial is None:
            raise RuntimeError('pyserial is not available. Install with `pip install pyserial`.')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'Opened serial {self.port} @ {self.baud} baud')
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except SerialException as e:  # pragma: no cover
            self.get_logger().fatal(f'Failed to open serial {self.port}: {e}')
            raise

        # ---- ROS I/O ----
        qos_cmd = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        qos_odom = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                              history=QoSHistoryPolicy.KEEP_LAST)

        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, qos_cmd)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, qos_odom)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.dt_odom = 1.0 / max(1.0, self.odom_rate)
        self.timer_odom = self.create_timer(self.dt_odom, self._tick_odom)
        self.timer_watchdog = self.create_timer(0.05, self._tick_watchdog)  # 20 Hz

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_L_m: Optional[float] = None
        self._last_R_m: Optional[float] = None
        self._last_odom_stamp = self.get_clock().now()
        self._last_cmd_time = time.monotonic()
        self._last_twist_lin = 0.0
        self._last_twist_ang = 0.0

        # Ensure motors are idle on start
        self._send({'T': 131, 'cmd': 0})

    # ------------------------ Serial protocol ---------------------------- #

    def _send(self, obj: dict) -> None:
        try:
            line = json.dumps(obj, separators=(',', ':')) + '\n'
            self.ser.write(line.encode('utf-8'))
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f'Serial write failed: {e}')

    def _read_json_line(self) -> Optional[dict]:
        try:
            raw = self.ser.readline()
            if not raw:
                return None
            # strip any non-json garbage before first '{'
            s = raw.decode('utf-8', errors='ignore')
            i = s.find('{')
            j = s.rfind('}')
            if i == -1 or j == -1 or j <= i:
                return None
            s = s[i:j+1]
            return json.loads(s)
        except json.JSONDecodeError:
            return None
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f'Serial read error: {e}')
            return None

    # ----------------------- ROS callbacks ------------------------------ #

    def _on_cmd_vel(self, msg: Twist) -> None:
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Convert v, w -> wheel tangential velocities [m/s]
        vL = v - (self.W * 0.5) * w
        vR = v + (self.W * 0.5) * w

        # Convert to RPM(1 unit is 0.1 RPM)
        rpmL = (vL / (2.0 * math.pi * self.R)) * 60.0
        rpmR = (vR / (2.0 * math.pi * self.R)) * 60.0

        if self.invL:
            rpmL = -rpmL
        if self.invR:
            rpmR = -rpmR

        rpmL = clamp(rpmL, -self.max_rpm, self.max_rpm)
        rpmR = clamp(rpmR, -self.max_rpm, self.max_rpm)

        self._send({'T': 1, 'L': int(rpmL * 10), 'R': int(rpmR * 10)})
        self._last_cmd_time = time.monotonic()
        self._last_twist_lin = v
        self._last_twist_ang = w

    def _tick_watchdog(self) -> None:
        # Stop wheels if we have not received /cmd_vel recently
        if time.monotonic() - self._last_cmd_time > self.cmd_timeout:
            # send zeros only once per timeout window to reduce bus load
            if abs(self._last_twist_lin) > 1e-6 or abs(self._last_twist_ang) > 1e-6:
                self._last_twist_lin = 0.0
                self._last_twist_ang = 0.0
                self._send({'T': 1, 'L': 0.0, 'R': 0.0})

    def _tick_odom(self) -> None:
        # Optionally poll the MCU for odometry
        if self.poll_odom and self.odom_poll_template:
            try:
                # Allow poll template to be plain JSON text
                self.ser.write((self.odom_poll_template + '\n').encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Failed to poll odom: {e}')

        data = self._read_json_line()
        if not isinstance(data, dict):
            return

        if self.left_key not in data or self.right_key not in data:
            return  # unrelated packet

        # Read cumulative travel for left/right wheel
        L = float(data[self.left_key])
        R = float(data[self.right_key])

        # Units -> meters
        if self.units == 'cm':
            L *= 0.01
            R *= 0.01
        elif self.units == 'm':
            pass
        else:
            # Unknown units, ignore
            return

        # If motors inverted, odometry counters may be inverted too
        if self.invL:
            L = -L
        if self.invR:
            R = -R

        # First sample init
        if self._last_L_m is None:
            self._last_L_m = L
            self._last_R_m = R
            self._last_odom_stamp = self.get_clock().now()
            return

        dL = L - self._last_L_m
        dR = R - self._last_R_m
        self._last_L_m = L
        self._last_R_m = R

        dC = 0.5 * (dL + dR)              # [m]
        dYaw = (dR - dL) / self.W         # [rad]

        # Integrate in body frame -> world frame
        cy = math.cos(self.yaw + dYaw / 2.0)
        sy = math.sin(self.yaw + dYaw / 2.0)
        self.x += dC * cy
        self.y += dC * sy
        self.yaw += dYaw

        # Timing
        now = self.get_clock().now()
        dt = (now - self._last_odom_stamp).nanoseconds * 1e-9
        self._last_odom_stamp = now
        vx = dC / dt if dt > 0.0 else 0.0
        wz = dYaw / dt if dt > 0.0 else 0.0

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        # Set simple diagonal covariances (tune for your platform)
        odom.pose.covariance[0] = self.cov_pose_xy
        odom.pose.covariance[7] = self.cov_pose_xy
        odom.pose.covariance[35] = self.cov_pose_yaw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance[0] = self.cov_twist_xy
        odom.twist.covariance[7] = self.cov_twist_xy
        odom.twist.covariance[35] = self.cov_twist_yaw

        self.pub_odom.publish(odom)

        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    # ------------------------- Lifecycle -------------------------------- #

    def destroy_node(self) -> bool:  # graceful stop
        try:
            self._send({'T': 131, 'cmd': 0})
            time.sleep(0.02)
            if hasattr(self, 'ser'):
                self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


# ------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = CobraflexDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
