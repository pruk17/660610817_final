#!/usr/bin/env python3
import sys
import os
sys.path.append('/home/chaiyapruk/venv_496/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import math

class SpeedControl(Node):
    def __init__(self):
        super().__init__('speed_ctrl_node')
        self.speed_pub = self.create_publisher(Vector3, '/speed_settings', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # ค่าเริ่มต้น 40% ตามโจทย์
        self.lin_v, self.ang_v = 0.4, 1.0
        self.lin_a, self.ang_a = 0.4, 0.4

        model_path = '/home/chaiyapruk/660610817_final/src/elephant_control/elephant_control/hand_landmarker.task'
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        options = vision.HandLandmarkerOptions(
            base_options=python.BaseOptions(model_asset_path=model_path),
            num_hands=2, # ต้องเป็น 2 เช่นกัน
            min_hand_detection_confidence=0.8
        )
        self.detector = vision.HandLandmarker.create_from_options(options)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        res = self.detector.detect(mp_image)
        status, color = "R: IDLE", (255, 255, 255)

        if res.hand_landmarks:
            for i in range(len(res.hand_landmarks)):
                if res.handedness[i][0].category_name == "Left": # มือขวาเรา
                    lm = res.hand_landmarks[i]
                    t_tip, i_tip, r_tip, p_tip = lm[4], lm[8], lm[16], lm[20]
                    i_mcp, p_mcp = lm[5], lm[17]
                    
                    is_i_up = i_tip.y < i_mcp.y - 0.08
                    is_p_up = p_tip.y < p_mcp.y - 0.08
                    is_r_up = r_tip.y < lm[13].y - 0.08
                    is_t_open = abs(t_tip.x - i_mcp.x) > 0.12
                    dx = i_tip.x - i_mcp.x
                    step = 0.01 if dx > 0.06 else -0.01 if dx < -0.06 else 0

                    if is_i_up: # นิ้วชี้ Priority 1
                        if not is_t_open: self.lin_v = max(0.1, min(1.0, self.lin_v + step)); status = f"SET LIN VEL: {self.lin_v:.2f}"
                        else: self.ang_v = max(0.5, min(2.5, self.ang_v + step)); status = f"SET ANG VEL: {self.ang_v:.2f}"
                        color = (0, 255, 255)
                    elif is_p_up: # นิ้วก้อย
                        self.lin_a = max(0.1, min(1.0, self.lin_a + step)); status = f"SET LIN ACC: {self.lin_a:.2f}"
                        color = (255, 0, 255)
                    elif is_r_up: # นิ้วนาง
                        self.ang_a = max(0.1, min(1.0, self.ang_a + step)); status = f"SET ANG ACC: {self.ang_a:.2f}"
                        color = (0, 255, 255)
                    break

        self.speed_pub.publish(Vector3(x=float(self.lin_v), y=float(self.ang_vel), z=0.0))
        cv2.rectangle(frame, (0, 0), (w, 60), (30, 30, 30), -1)
        cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow('Right Hand Speed Control', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = SpeedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()