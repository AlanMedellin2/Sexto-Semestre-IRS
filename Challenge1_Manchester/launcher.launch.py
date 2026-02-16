from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'Challenge1_Manchester',
            executable = 'signal_generator_executable',
            output = 'screen'),
        Node(
            package = 'Challenge1_Manchester',
            executable = 'process_executable',
            output = 'screen')
    ])
