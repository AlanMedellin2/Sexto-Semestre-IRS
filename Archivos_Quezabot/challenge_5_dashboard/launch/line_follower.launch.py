from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='challenge_5_dashboard',
            executable='line_detector.py',
            name='line_detector',
            output='screen'
        ),

        Node(
            package='challenge_5_dashboard',
            executable='color_detector_node.py',
            name='color_detector_node',
            output='screen'
        ),

        Node(
            package='challenge_5_dashboard',
            executable='line_follower.py',
            name='pid_controller',
            output='screen'
        )

    ])
