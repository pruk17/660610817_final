import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ScanDistanceReceiver(Node):

    def __init__(self):

        super().__init__('scan_distance_receiver')

        self.subscription = self.create_subscription(
            Float32,
            '/scan_distance',
            self.distance_callback,
            10
        )

        self.get_logger().info('Listening to /scan_distance_300mm')


    def distance_callback(self, msg):

        distance = msg.data

        # convert to mm for easier reading
        distance_mm = distance * 1000

        self.get_logger().info(
            f'Obstacle detected at {distance_mm:.1f} mm'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ScanDistanceReceiver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()