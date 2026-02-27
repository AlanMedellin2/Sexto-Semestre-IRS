from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    motor_node_1 = Node(
        name="motor_sys_1",
        package='motor_control_pkg',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group1",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.78,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )

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
            'kp': 2.0,
            'ki': 0.0,  
            'kd': 0.0,  
        }]
    )

    server_node_1 = Node(
        name="server_node_1",
        package='motor_control_pkg',
        executable='server',
        emulate_tty=True,
        output='screen',
        namespace="group1",
    )

    motor_node_2 = Node(
        name="motor_sys_2",
        package='motor_control_pkg',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group2",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.78,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )

    sp_node_2 = Node(
        name="sp_gen_2",
        package='motor_control_pkg',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group2",
        parameters=[{'signal_type': 'square'}]  
    )

    control_node_2 = Node(
        name="control_node_2",
        package='motor_control_pkg',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group2",
        parameters=[{
            'kp': 2.0,
            'ki': 0.0,
            'kd': 0.0,
        }]
    )

    server_node_2 = Node(
        name="server_node_2",
        package='motor_control_pkg',
        executable='server',
        emulate_tty=True,
        output='screen',
        namespace="group2",
    )

    motor_node_3 = Node(
        name="motor_sys_3",
        package='motor_control_pkg',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group3",
        parameters=[{
            'sample_time': 0.02,
            'sys_gain_K': 1.78,
            'sys_tau_T': 0.5,
            'initial_conditions': 0.0,
        }]
    )

    sp_node_3 = Node(
        name="sp_gen_3",
        package='motor_control_pkg',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group3",
        parameters=[{'signal_type': 'triangle'}]  
    )

    control_node_3 = Node(
        name="control_node_3",
        package='motor_control_pkg',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group3",
        parameters=[{
            'kp': 2.0,
            'ki': 0.0,
            'kd': 0.0,
        }]
    )

    server_node_3 = Node(
        name="server_node_3",
        package='motor_control_pkg',
        executable='server',
        emulate_tty=True,
        output='screen',
        namespace="group3",
    )

    l_d = LaunchDescription([
        motor_node_1, sp_node_1, control_node_1, server_node_1,
        motor_node_2, sp_node_2, control_node_2, server_node_2,
        motor_node_3, sp_node_3, control_node_3, server_node_3,
    ])

    return l_d
