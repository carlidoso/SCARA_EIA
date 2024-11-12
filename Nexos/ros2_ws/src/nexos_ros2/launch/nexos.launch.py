from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nexos_ros2',
            executable='nexos_app',
        ),
        Node(
            package='nexos_ros2',
            executable='cinematica_inversa',
        ),
    ])