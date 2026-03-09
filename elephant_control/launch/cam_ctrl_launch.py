from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        
    camera_node = Node(
        package='elephant_control', 
        executable='cam_publisher',
        name='camera_pub',
        output='screen'
    )
    
    teleop_node = Node(
        package='elephant_control', 
        executable='cam_teleop',
        name='cam_teleop_control',
        output='screen',
        # parameters=[{'use_sim_time': False}] 
    )

    reset_speed_server = Node(
        package='elephant_control',
        executable='set_speed_server',
        name='set_speed_service_server',
        output='screen'
    )

    laptop_UDP_receiver = Node(
        package='elephant_control',
        executable='UDP_run',
        name='UDP_receiver_runner',
        output='screen'
    )
    


    return LaunchDescription([
        camera_node,
        teleop_node,
        reset_speed_server,
        laptop_UDP_receiver
        
    ])