#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class SetSpeedService(Node):

    def __init__(self):
        super().__init__('set_speed_service_server')

        self.srv = self.create_service(
            Trigger,
            'reset_speed',
            self.reset_callback
        )

        # publisher ไป teleop
        self.pub = self.create_publisher(
            Bool,
            '/reset_speed_cmd',
            10
        )

        self.get_logger().info("Speed Setter Service Ready")

    def reset_callback(self, request, response):

        self.get_logger().info("request received")

        msg = Bool()
        msg.data = True

        self.pub.publish(msg)

        response.success = True
        response.message = "command accepted"

        return response


def main():

    rclpy.init()

    node = SetSpeedService()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()