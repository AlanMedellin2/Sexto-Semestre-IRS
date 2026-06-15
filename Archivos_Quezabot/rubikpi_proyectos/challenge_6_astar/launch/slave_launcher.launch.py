from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='challenge_6_astar',
            executable='camera_raw_node.py',
            output='screen'
        ),

        Node(
            package='challenge_6_astar',
            executable='yolo_decision_node.py',
            name='yolo_decision_node',
            output='screen',
            prefix='/home/ubuntu/ros2_ws/src/challenge_6/yolo_ros_env/bin/python'
        ),

        Node(
            package='challenge_6_astar',
            executable='ultrasonic_obstacle.py',
            output='screen'
        )

    ])
