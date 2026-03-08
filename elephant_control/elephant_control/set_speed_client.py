#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SetSpeedClient(Node):

    def __init__(self):
        super().__init__('set_speed_client')

        self.client = self.create_client(
            Trigger,
            'reset_speed'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.request = Trigger.Request()

    def send_request(self):

        future = self.client.call_async(self.request)

        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):

    rclpy.init(args=args)

    node = SetSpeedClient()

    response = node.send_request()

    if response is not None:
        node.get_logger().info(
            f"Success: {response.success}, Message: {response.message}"
        )
    else:
        node.get_logger().error("Service call failed")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()