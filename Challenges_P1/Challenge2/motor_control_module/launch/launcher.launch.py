from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'motor_control_module',
            executable = 'dc_motor',
            output = 'screen'),
        Node(
            package = 'motor_control_module',
            executable = 'set_point',
            output = 'screen'),
        Node(
            package = 'motor_control_module',
            executable = 'server',
            output = 'screen'),
        Node(
            package = 'motor_control_module',
            executable = 'controller',
            output = 'screen')
    ])
