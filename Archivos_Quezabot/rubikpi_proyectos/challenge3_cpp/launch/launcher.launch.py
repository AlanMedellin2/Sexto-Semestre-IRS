from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('challenge3_cpp'),
        'config',
        'a_star_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='challenge3_cpp',
            executable='node_odometry.py',
            output='screen'
        ),

        Node(
            package='challenge3_cpp',
            executable='node_extra.py',
            name='a_star_goals_publisher',
            parameters=[params_file],
            output='screen'
        ),

        Node(
            package='challenge3_cpp',
            executable='error.py',
            output='screen'
        ),

        Node(
            package='challenge3_cpp',
            executable='closed_loop_ctrl.py',
            output='screen'
        )
        ])
