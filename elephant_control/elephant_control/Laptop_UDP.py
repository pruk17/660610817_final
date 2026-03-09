#!/usr/bin/env python3
"""
node2_udp_gateway_laptop.py
รันบน Laptop  (ROS_DOMAIN_ID=79)

Flow ส่ง:  /cmd_vel_command → UDP:15000 → AGV
Flow รับ:  AGV UDP:15001    → /scan_distance (Domain 79)

run:
  ROS_DOMAIN_ID=79 python3 node2_udp_gateway_laptop.py \
    --ros-args -p robot_ip:=10.244.171.178
"""

import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class UdpGatewayLaptop(Node):

    def __init__(self):
        super().__init__('udp_gateway_laptop')

        self.declare_parameter('robot_ip',  '10.244.171.178')  # IP ของ AGV
        self.declare_parameter('cmd_port',  15000)
        self.declare_parameter('data_port', 15001)

        self.robot_ip  = self.get_parameter('robot_ip').value
        self.cmd_port  = self.get_parameter('cmd_port').value
        self.data_port = self.get_parameter('data_port').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── ส่ง CMD ──────────────────────────────────────────────
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(
            Twist, '/cmd_vel_command', self._cmd_cb, qos)

        # ── รับ DIST จาก AGV ─────────────────────────────────────
        self.dist_pub = self.create_publisher(Float32, '/scan_distance', qos)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(('0.0.0.0', self.data_port))
        threading.Thread(target=self._data_loop, daemon=True).start()

        self.get_logger().info(
            f'[Laptop | DOMAIN 79] /cmd_vel_command → UDP {self.robot_ip}:{self.cmd_port}')
        self.get_logger().info(
            f'[Laptop | DOMAIN 79] UDP:{self.data_port} → /scan_distance')

    def _cmd_cb(self, msg: Twist):
        data = f'{msg.linear.x:.4f} {msg.linear.y:.4f} {msg.angular.z:.4f}'
        self.sock_send.sendto(data.encode(), (self.robot_ip, self.cmd_port))

    def _data_loop(self):
        self.sock_recv.settimeout(1.0)
        while rclpy.ok():
            try:
                data, _ = self.sock_recv.recvfrom(2048)
                val = float(data.decode().strip())
                if val == val and val != float('inf'):   # NaN / inf check
                    msg = Float32()
                    msg.data = val
                    self.dist_pub.publish(msg)
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().warn(f'Data recv error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UdpGatewayLaptop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()