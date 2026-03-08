#!/usr/bin/env python3
import sys
import os
# บังคับใช้ venv
sys.path.append('/home/chaiyapruk/venv_496/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import math

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class SpeedManager(Node):
    def __init__(self):
        super().__init__('speed_manager_node')
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # โหลดโมเดล (ใช้ Absolute Path ตามเดิม)
        model_path = '/home/chaiyapruk/660610817_final/src/elephant_control/elephant_control/hand_landmarker.task'
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1, # สนใจแค่มือเดียวที่เจอ (เราจะกรองว่าเป็นมือขวา)
            min_hand_detection_confidence=0.8
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

        # --- ค่าเริ่มต้น (40%) ---
        self.linear_vel = 0.4
        self.angular_vel = 0.4
        self.linear_accel = 0.4
        self.angular_accel = 0.4
        
        self.get_logger().info("Speed Manager Node Started - Right Hand Control")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        detection_result = self.detector.detect(mp_image)
        
        status = "RIGHT HAND: WAITING"
        color = (200, 200, 200)

        if detection_result.hand_landmarks and detection_result.handedness:
            # MediaPipe มองมือขวา (หลัง Flip) เป็น "Left" 
            # (เนื่องจากคุณบอกว่ามือซ้ายระบบเห็นเป็น Right ดังนั้นมือขวาระบบจะเห็นเป็น Left)
            hand_label = detection_result.handedness[0][0].category_name
            
            if hand_label == "Left": 
                landmarks = detection_result.hand_landmarks[0]
                
                # ดึงจุดที่ใช้งาน
                wrist = landmarks[0]
                thumb_tip = landmarks[4]
                index_mcp = landmarks[5]
                index_tip = landmarks[8]
                ring_tip = landmarks[16]
                pinky_tip = landmarks[20]

                # เช็คการกางนิ้วโป้ง (เทียบกับข้อมือ)
                thumb_dist = math.sqrt((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2)
                is_thumb_open = thumb_dist > 0.18
                
                # เช็คสถานะการชูนัด
                is_index_up = index_tip.y < index_mcp.y - 0.08
                is_pinky_up = pinky_tip.y < wrist.y - 0.15
                is_ring_up = ring_tip.y < wrist.y - 0.15
                
                # ความเอียง (dX) สำหรับการเพิ่ม/ลดค่า
                dx = index_tip.x - index_mcp.x
                change = 0.0 # ค่าที่จะปรับเปลี่ยน
                if dx > 0.06: change = 0.01   # เอียงขวา เพิ่ม
                elif dx < -0.06: change = -0.01 # เอียงซ้าย ลด

                # --- Logic การเลือกโหมด (Priority: Index > Pinky > Ring) ---
                if is_index_up:
                    if not is_thumb_open:
                        self.linear_vel = max(0.0, min(1.0, self.linear_vel + change))
                        status = f"ADJUST: LINEAR VEL ({self.linear_vel:.2%})"
                    else:
                        self.angular_vel = max(0.0, min(1.0, self.angular_vel + change))
                        status = f"ADJUST: ANGULAR VEL ({self.angular_vel:.2%})"
                    color = (0, 255, 255)
                
                elif is_pinky_up:
                    self.linear_accel = max(0.0, min(1.0, self.linear_accel + change))
                    status = f"ADJUST: LINEAR ACCEL ({self.linear_accel:.2%})"
                    color = (255, 0, 255)
                
                elif is_ring_up:
                    if is_thumb_open:
                        self.angular_accel = max(0.0, min(1.0, self.angular_accel + change))
                        status = f"ADJUST: ANGULAR ACCEL ({self.angular_accel:.2%})"
                    color = (255, 255, 0)

                # วาดจุดเพื่อ Debug
                for id in [4, 8, 16, 20]:
                    lm = landmarks[id]
                    cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 8, color, -1)

        # แสดงผลสี่เหลี่ยมตามที่คุณต้องการ (ยาวขึ้นและบางลง)
        cv2.rectangle(frame, (0, 0), (640, 60), (30, 30, 30), -1)
        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow('Right Hand Speed Manager', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = SpeedManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()