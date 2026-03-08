#!/usr/bin/env python3
import sys
sys.path.append('/home/chaiyapruk/venv_496/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import mediapipe as mp
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

        self.publisher_ = self.create_publisher(
            Twist, '/cmd_vel_command', qos_profile)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # speed values
        self.base_linear_speed = 0.4
        self.base_angular_speed = 0.4

        self.frame_id = 0

        model_path = '/home/chaiyapruk/660610817_final/src/elephant_control/elephant_control/hand_landmarker.task'

        base_options = python.BaseOptions(model_asset_path=model_path)

        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=2,
            running_mode=vision.RunningMode.VIDEO,
            min_hand_detection_confidence=0.35,
            min_hand_presence_confidence=0.35,
            min_tracking_confidence=0.35
        )

        self.detector = vision.HandLandmarker.create_from_options(options)

        # smoothing (separate hands)
        self.smooth_thumb_dist_l = 0.0
        self.smooth_dx_l = 0.0

        self.smooth_thumb_dist_r = 0.0
        self.smooth_dx_r = 0.0

        self.alpha = 0.5

        self.get_logger().info("Dual Hand Teleop Controller Started")

    def distance(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        )

        detection_result = self.detector.detect_for_video(
            mp_image,
            self.frame_id
        )

        self.frame_id += 1

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        status_l = "L: STOP"
        status_r = "R: SPEED CTRL"

        color_l = (0, 0, 255)
        color_r = (255, 255, 255)

        if detection_result.hand_landmarks:

            for i in range(len(detection_result.hand_landmarks)):

                lm = detection_result.hand_landmarks[i]
                hand_label = detection_result.handedness[i][0].category_name

                # LEFT HAND (movement)
                if hand_label == "Right":

                    t_tip = lm[4]
                    i_mcp = lm[5]
                    i_tip = lm[8]

                    wrist = lm[0]
                    middle_mcp = lm[9]

                    hand_size = self.distance(wrist, middle_mcp)
                    thumb_dist = self.distance(t_tip, i_mcp)

                    dx = i_tip.x - i_mcp.x

                    self.smooth_thumb_dist_l = (
                        self.alpha * thumb_dist +
                        (1 - self.alpha) * self.smooth_thumb_dist_l
                    )

                    self.smooth_dx_l = (
                        self.alpha * dx +
                        (1 - self.alpha) * self.smooth_dx_l
                    )

                    is_i_up = i_tip.y < i_mcp.y - 0.08
                    is_t_open = self.smooth_thumb_dist_l > (hand_size * 0.35)

                    if is_i_up:

                        if is_t_open:

                            linear_x = self.base_linear_speed * 0.5

                            if self.smooth_dx_l < -0.04:
                                angular_z = self.base_angular_speed
                                status_l = "L: ROTATE L"

                            elif self.smooth_dx_l > 0.04:
                                angular_z = -self.base_angular_speed
                                status_l = "L: ROTATE R"

                            else:
                                status_l = "L: FORWARD (ROT)"

                        else:

                            linear_x = self.base_linear_speed

                            if self.smooth_dx_l < -0.05:
                                linear_y = self.base_linear_speed
                                status_l = "L: SLIDE L"

                            elif self.smooth_dx_l > 0.05:
                                linear_y = -self.base_linear_speed
                                status_l = "L: SLIDE R"

                            else:
                                status_l = "L: FORWARD"

                        color_l = (0,255,0)

                    elif is_t_open:

                        if i_tip.y > i_mcp.y - 0.06:

                            linear_x = -self.base_linear_speed
                            status_l = "L: REVERSE"
                            color_l = (255,0,255)

                    cv2.circle(frame,(int(t_tip.x*w),int(t_tip.y*h)),10,(255,0,255),-1)
                    cv2.circle(frame,(int(i_tip.x*w),int(i_tip.y*h)),10,(0,255,0),-1)

                # RIGHT HAND (speed adjust only)
                elif hand_label == "Left":

                    ti_tip = lm[4]
                    in_mcp = lm[5]
                    in_tip = lm[8]

                    wrist = lm[0]
                    middle_mcp = lm[9]

                    hand_size = self.distance(wrist, middle_mcp)
                    thumb_dist = self.distance(ti_tip, in_mcp)

                    dx = in_tip.x - in_mcp.x

                    self.smooth_thumb_dist_r = (
                        self.alpha * thumb_dist +
                        (1 - self.alpha) * self.smooth_thumb_dist_r
                    )

                    self.smooth_dx_r = (
                        self.alpha * dx +
                        (1 - self.alpha) * self.smooth_dx_r
                    )

                    is_index_up = in_tip.y < in_mcp.y - 0.08
                    is_thumb_open = self.smooth_thumb_dist_r > (hand_size * 0.35)

                    if is_index_up:

                        color_r = (0,255,255)

                        if not is_thumb_open:

                            if self.smooth_dx_r > 0.04:
                                self.base_linear_speed = min(1.0, self.base_linear_speed + 0.02)

                            elif self.smooth_dx_r < -0.04:
                                self.base_linear_speed = max(0.1, self.base_linear_speed - 0.02)

                            status_r = "R: LIN SPEED"

                        else:

                            if self.smooth_dx_r > 0.04:
                                self.base_angular_speed = min(2.5, self.base_angular_speed + 0.02)

                            elif self.smooth_dx_r < -0.04:
                                self.base_angular_speed = max(0.2, self.base_angular_speed - 0.02)

                            status_r = "R: ANG SPEED"

                    cv2.circle(frame,(int(in_tip.x*w),int(in_tip.y*h)),10,(0,255,255),-1)
                    cv2.circle(frame,(int(ti_tip.x*w),int(ti_tip.y*h)),10,(255,0,255),-1)

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)

        self.publisher_.publish(twist)

        cv2.rectangle(frame,(0,0),(w,90),(30,30,30),-1)

        cv2.putText(frame,status_l,(20,35),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,color_l,2)

        cv2.putText(frame,status_r,(int(w*0.45),35),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,color_r,2)

        cv2.putText(
            frame,
            f"LIN:{self.base_linear_speed:.2f}   ANG:{self.base_angular_speed:.2f}",
            (20,70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (200,200,200),
            2
        )

        cv2.imshow('Mecanum Dual Hand Master',frame)
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