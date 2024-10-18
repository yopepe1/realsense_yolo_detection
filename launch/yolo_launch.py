from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo',  
            executable='yolo_image_node',
            output='screen'),
        Node(
            package='yolo', 
            executable='control_node',
            output='screen'),
    ])
