#!/usr/bin/env python3
"""
set_speed_service_server.py  (แก้ไขจากเดิม)
รันบน Laptop  ROS_DOMAIN_ID=79

Services ที่รองรับ:
  'reset_speed' → /reset_speed_cmd (Bool) — ความเร็วปกติ (เดิมมีอยู่แล้ว)
  'slow_speed'  → /slow_speed_cmd  (Bool) — linear=0.10, angular=0.2
  'stop_speed'  → /stop_speed_cmd  (Bool) — linear=0, angular=0
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class SetSpeedService(Node):

    def __init__(self):
        super().__init__('set_speed_service_server')

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, 'reset_speed', self.reset_callback)
        self.create_service(Trigger, 'slow_speed',  self.slow_callback)
        self.create_service(Trigger, 'stop_speed',  self.stop_callback)

        # ── Publishers ───────────────────────────────────────────
        self.reset_pub = self.create_publisher(Bool, '/reset_speed_cmd', 10)
        self.slow_pub  = self.create_publisher(Bool, '/slow_speed_cmd',  10)
        self.stop_pub  = self.create_publisher(Bool, '/stop_speed_cmd',  10)

        self.get_logger().info('Speed Service Ready: reset_speed | slow_speed | stop_speed')

    # ── Callbacks ────────────────────────────────────────────────

    def reset_callback(self, request, response):
        self.get_logger().info('[reset_speed] → /reset_speed_cmd')
        self._pub(self.reset_pub)
        response.success = True
        response.message = 'reset speed command accepted'
        return response

    def slow_callback(self, request, response):
        self.get_logger().warn('[slow_speed] linear=0.10  angular=0.2 → /slow_speed_cmd')
        self._pub(self.slow_pub)
        response.success = True
        response.message = 'slow speed command accepted'
        return response

    def stop_callback(self, request, response):
        self.get_logger().error('[stop_speed] linear=0  angular=0 → /stop_speed_cmd')
        self._pub(self.stop_pub)
        response.success = True
        response.message = 'stop command accepted'
        return response

    def _pub(self, publisher):
        msg = Bool()
        msg.data = True
        publisher.publish(msg)


def main():
    rclpy.init()
    node = SetSpeedService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()