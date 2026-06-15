from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'challenge3_cpp',
            executable = 'rotate_node.py',
            output = 'screen'),
        Node(
            package = 'challenge3_cpp',
            executable = 'node_odometry.py',
            output = 'screen'),
    ])
