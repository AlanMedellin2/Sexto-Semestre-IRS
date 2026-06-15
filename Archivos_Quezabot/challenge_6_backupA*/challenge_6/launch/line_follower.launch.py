from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ====================================
        # Nodo visión
        # ====================================
        Node(
            package='challenge_5',
            executable='line_detector.py',
            name='line_detector',
            output='screen'
        ),

        # ====================================
        # Nodo PID
        # ====================================
        Node(
            package='challenge_5',
            executable='line_follower.py',
            name='pid_controller',
            output='screen'
        )

    ])
