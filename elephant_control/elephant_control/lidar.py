#!/usr/bin/env python3
"""
lidar_scanner.py — Run on Laptop (ROS_DOMAIN_ID=79)
Subscribes : /scan_distance (String "distance_m,angle_deg")
Calls service on zone change:
  CLEAR   -> call 'reset_speed'
  WARNING -> call 'slow_speed'
  STOP    -> call 'stop_speed'
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Distance thresholds (meters)
WARNING_DIST = 0.400   # < 250 mm -> slow
STOP_DIST    = 0.300   # < 200 mm -> stop
ZONE_HYST    = 0.020   # hysteresis to prevent oscillation
CONTROL_HZ   = 40


class LidarScanner(Node):

    def __init__(self):
        super().__init__('lidar_distance_scan')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/scan_distance', self._scan_cb, qos)

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

        self.get_logger().info('════ LIDAR SCANNER ════')
        self.get_logger().info(f'WARNING < {WARNING_DIST*1000:.0f} mm -> slow_speed')
        self.get_logger().info(f'STOP    < {STOP_DIST*1000:.0f} mm -> stop_speed')

    def _scan_cb(self, msg: String):
        try:
            self.latest_dist = float(msg.data.split(',')[0])
        except Exception:
            pass

    def _control_loop(self):
        if self.latest_dist is None:
            return

        dist = self.latest_dist

        # Zone detection
        if dist <= STOP_DIST:
            zone = 'STOP'
        elif dist <= WARNING_DIST - ZONE_HYST:
            zone = 'WARNING'
        else:
            zone = 'CLEAR'

        # Only call service when zone changes
        if zone == self.last_zone:
            return

        dist_mm = dist * 1000

        if zone == 'CLEAR':
            self.get_logger().info(f'[CLEAR] {dist_mm:.0f} mm -> reset_speed')
            self._call(self.reset_client, 'reset_speed')
        elif zone == 'WARNING':
            self.get_logger().warn(f'[WARNING] {dist_mm:.0f} mm -> slow_speed')
            self._call(self.slow_client, 'slow_speed')
        elif zone == 'STOP':
            self.get_logger().error(f'[STOP] {dist_mm:.0f} mm -> stop_speed')
            self._call(self.stop_client, 'stop_speed')

        self.last_zone = zone

    def _call(self, client, name: str):
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._done_cb(f, name))

    def _done_cb(self, future, name: str):
        try:
            result = future.result()
            self.get_logger().info(f'{name} -> {result.message}')
        except Exception as e:
            self.get_logger().error(f'{name} error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanner()
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