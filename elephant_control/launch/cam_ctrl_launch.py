from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Node สำหรับเปิดกล้องและ Publish ภาพ
        Node(
            package='elephant_control', 
            executable='cam_publisher',
            name='camera_pub',
            output='screen'
        ),
        
        # 2. Node สำหรับประมวลผล MediaPipe และควบคุมหุ่นยนต์
        Node(
            package='elephant_control', 
            executable='cam_teleop',
            name='cam_teleop_control',
            output='screen',
            parameters=[{'use_sim_time': False}] # ตั้งค่าพื้นฐานหากจำเป็น
        ),

        # Node(
        #     package='elephant_control', 
        #     executable='speed_ctrl',
        #     name='speed_control',
        #     output='screen'
        # ),
    ])