from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_ekf',
            executable='ackermann_ekf',
            name='ackermann_ekf_node',
            output='screen'
        )
    ])