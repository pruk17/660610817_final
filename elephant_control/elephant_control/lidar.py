#!/usr/bin/env python3
"""
lidar_guard_pro.py
รันบน Laptop  ROS_DOMAIN_ID=79

รับ : /scan_distance (Float32)
call service เมื่อ zone เปลี่ยน:
  CLEAR   → call 'reset_speed'
  WARNING → call 'slow_speed'   (linear=0.10, angular=0.2)
  STOP    → call 'stop_speed'   (linear=0, angular=0)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

# ── ระยะ threshold (เมตร) ────────────────────────────────────
WARNING_DIST = 0.35   # ใกล้ชน → slow
STOP_DIST    = 0.20   # จะชนแล้ว → stop
ZONE_HYST    = 0.02   # hysteresis กัน oscillation
CONTROL_HZ   = 40


class LidarGuardPro(Node):

    def __init__(self):
        super().__init__('lidar_guard_pro')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32, '/scan_distance', self._scan_cb, qos)

        # ── Service clients (ทั้ง 3 ชี้ไปที่ server เดียวกัน) ────
        self.reset_client = self.create_client(Trigger, 'reset_speed')
        self.slow_client  = self.create_client(Trigger, 'slow_speed')
        self.stop_client  = self.create_client(Trigger, 'stop_speed')

        self.get_logger().info('Waiting for set_speed_service_server...')
        self.reset_client.wait_for_service()
        self.slow_client.wait_for_service()
        self.stop_client.wait_for_service()
        self.get_logger().info('Service server ready!')

        self.latest_dist = None
        self.last_zone   = 'CLEAR'

        self.create_timer(1.0 / CONTROL_HZ, self._control_loop)

        self.get_logger().info('════ LIDAR GUARD PRO ════')
        self.get_logger().info(f'WARNING ≤ {WARNING_DIST*1000:.0f} mm → slow_speed')
        self.get_logger().info(f'STOP    ≤ {STOP_DIST*1000:.0f} mm → stop_speed')

    def _scan_cb(self, msg: Float32):
        self.latest_dist = msg.data

    def _control_loop(self):
        if self.latest_dist is None:
            return

        dist = self.latest_dist

        # ── Zone detection ────────────────────────────────────────
        if dist <= STOP_DIST:
            zone = 'STOP'
        elif dist <= WARNING_DIST - ZONE_HYST:
            zone = 'WARNING'
        else:
            zone = 'CLEAR'

        # ── Call service เฉพาะเมื่อ zone เปลี่ยน ─────────────────
        if zone == self.last_zone:
            return

        dist_mm = dist * 1000

        if zone == 'CLEAR':
            self.get_logger().info(f'[CLEAR] {dist_mm:.0f} mm → reset_speed')
            self._call(self.reset_client, 'reset_speed')

        elif zone == 'WARNING':
            self.get_logger().warn(f'[WARNING] {dist_mm:.0f} mm → slow_speed')
            self._call(self.slow_client, 'slow_speed')

        elif zone == 'STOP':
            self.get_logger().error(f'[STOP] {dist_mm:.0f} mm → stop_speed')
            self._call(self.stop_client, 'stop_speed')

        self.last_zone = zone

    def _call(self, client, name: str):
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._done_cb(f, name))

    def _done_cb(self, future, name: str):
        try:
            result = future.result()
            self.get_logger().info(f'{name} → {result.message}')
        except Exception as e:
            self.get_logger().error(f'{name} error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarGuardPro()
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