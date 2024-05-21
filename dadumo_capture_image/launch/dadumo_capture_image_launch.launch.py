#dadumo_capture_image_launch.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dadumo_capture_image',
            executable='capturar',
            output='screen'),
    ])