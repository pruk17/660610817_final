#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('cam_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("Camera Publisher Started!")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

    def __del__(self):
        self.cap.release()

def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()