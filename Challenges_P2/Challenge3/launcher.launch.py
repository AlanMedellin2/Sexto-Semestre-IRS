from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'puzzlebot_queza',
            executable = 'error.py',
            output = 'screen'),
        Node(
            package = 'puzzlebot_queza',
            executable = 'node_extra.py',
            output = 'screen'),
        Node(
            package = 'puzzlebot_queza',
            executable = 'closed_loop_ctrl.py',
            output = 'screen'),
    ])
