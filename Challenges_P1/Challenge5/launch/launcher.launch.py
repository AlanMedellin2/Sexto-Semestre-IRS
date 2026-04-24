from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sp_node_1 = Node(
        name="sp_gen_1",
        package='motor_control_pkg',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group1",
        parameters=[{'signal_type': 'sine'}]  
    )

    control_node_1 = Node(
        name="control_node_1",
        package='motor_control_pkg',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group1",
        parameters=[{
            'kp': 0.5,
            'ki': 1.4,  
            'kd': 0.0,  
        }]
    )

    l_d = LaunchDescription([
        sp_node_1,        #
        control_node_1,
    ])

    return l_d
