from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='challenge_6',
            executable='line_detector.py',
            name='line_detector',
            output='screen'
        ),

        Node(
            package='challenge_6',
            executable='yolo_decision_node.py',
            name='yolo_decision_node',
            output='screen',
            prefix='/home/ubuntu/ros2_ws/src/challenge_6/yolo_ros_env/bin/python'
        ),

        Node(
            package='challenge_6',
            executable='line_follower.py',
            name='pid_controller',
            output='screen'
        )

    ])
