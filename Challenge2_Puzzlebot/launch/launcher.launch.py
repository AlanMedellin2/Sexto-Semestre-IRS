from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    control_node = Node(
        package='motor_control_pkg',
        executable='control_node',
        name='open_loop_controller',
        output='screen'
    )

    return LaunchDescription([
        control_node
    ])
