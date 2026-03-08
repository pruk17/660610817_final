#!/usr/bin/env python3
import sys
import os
sys.path.append('/home/chaiyapruk/venv_496/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import math

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class CamTeleop(Node):
    def __init__(self):
        super().__init__('cam_teleop_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_command', qos_profile)
        
        # เปลี่ยนมา Subscribe ภาพแทน
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # การหา Path โมเดลอิงตามโครงสร้างที่คุณใช้งานได้ปกติ
        model_path = '/home/chaiyapruk/660610817_final/src/elephant_control/elephant_control/hand_landmarker.task'
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"notfound Path: {model_path}")
            return

        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.8,
            min_hand_presence_confidence=0.8
        )
        self.detector = vision.HandLandmarker.create_from_options(options)
        
        # --- ตัวแปรสำหรับกรองสัญญาณ ---
        self.smooth_thumb_dist = 0.0
        self.smooth_dx = 0.0
        self.alpha = 0.5  
        
        self.get_logger().info("Mecanum Control V5: Subscriber Mode Active!")

    def image_callback(self, msg):
        # รับภาพจาก Topic และแปลงเป็น OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Logic V5 เดิมของคุณทั้งหมด
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        detection_result = self.detector.detect(mp_image)
        
        linear_x, linear_y, angular_z = 0.0, 0.0, 0.0
        status = "STOP / NO HAND"
        color = (0, 0, 255)
        
        if detection_result.hand_landmarks and detection_result.handedness:
            hand_label = detection_result.handedness[0][0].category_name
            if hand_label == "Right": 
                landmarks = detection_result.hand_landmarks[0]
                thumb_tip, index_mcp, index_tip = landmarks[4], landmarks[5], landmarks[8]

                raw_thumb_dist = math.sqrt((thumb_tip.x - index_mcp.x)**2 + (thumb_tip.y - index_mcp.y)**2)
                raw_dx = index_tip.x - index_mcp.x

                self.smooth_thumb_dist = (self.alpha * raw_thumb_dist) + ((1 - self.alpha) * self.smooth_thumb_dist)
                self.smooth_dx = (self.alpha * raw_dx) + ((1 - self.alpha) * self.smooth_dx)

                is_index_up = index_tip.y < index_mcp.y - 0.08
                is_thumb_open = self.smooth_thumb_dist > 0.10

                if is_index_up:
                    if is_thumb_open:
                        linear_x = 0.2
                        if self.smooth_dx < -0.04: status, angular_z = "ROTATE LEFT", 1.0
                        elif self.smooth_dx > 0.04: status, angular_z = "ROTATE RIGHT", -1.0
                        else: status = "FORWARD (READY TO ROTATE)"
                    else:
                        linear_x = 0.4
                        if self.smooth_dx < -0.05: status, linear_y = "SLIDE LEFT", 0.4
                        elif self.smooth_dx > 0.05: status, linear_y = "SLIDE RIGHT", -0.4
                        else: status = "FORWARD"
                    color = (0, 255, 0)
                elif is_thumb_open:
                    if index_tip.y > index_mcp.y - 0.06:
                        linear_x, status = -0.4, "REVERSE"
                        color = (255, 0, 255)

                # Drawing
                pt1 = (int(thumb_tip.x * w), int(thumb_tip.y * h))
                pt2 = (int(index_mcp.x * w), int(index_mcp.y * h))
                cv2.line(frame, pt1, pt2, (255, 255, 0), 2)
                cv2.circle(frame, pt1, 8, (255, 0, 255), -1)
                cv2.circle(frame, pt2, 8, (255, 255, 255), -1)
                cv2.circle(frame, (int(index_tip.x * w), int(index_tip.y * h)), 8, (0, 255, 0), -1)

        # Publish Twist
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = float(linear_x), float(linear_y), float(angular_z)
        self.publisher_.publish(twist)

        # GUI
        cv2.rectangle(frame, (0, 0), (640, 60), (30, 30, 30), -1)
        cv2.putText(frame, f"STATUS: {status}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.imshow('Mecanum Control V5 (Sub)', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CamTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()