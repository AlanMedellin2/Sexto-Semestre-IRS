from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mpc_turtlebot',
            executable = 'mpc_controller_casadi',
            output = 'screen'),
        Node(
            package = 'mpc_turtlebot',
            executable = 'path_mpc_casadi',
            output = 'screen')
    ])
