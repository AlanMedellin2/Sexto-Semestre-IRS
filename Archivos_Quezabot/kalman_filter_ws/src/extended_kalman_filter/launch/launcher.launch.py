from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'extended_kalman_filter',
            executable = 'error.py',
            output = 'screen'),
        Node(
            package = 'extended_kalman_filter',
            executable = 'ekf.py',
            output = 'screen'),
        Node(
            package = 'extended_kalman_filter',
            executable = 'node_extra.py',
            output = 'screen'),
        Node(
            package = 'extended_kalman_filter',
            executable = 'closed_loop_ctrl.py',
            output = 'screen'),
        Node(
            package = 'extended_kalman_filter',
            executable = 'node_odometry.py',
            output = 'screen'),
    ])
