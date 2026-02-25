from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node_1 = Node(name="motor_sys_1",
                        package='motor_control',
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

    sp_node_1 = Node(name="sp_gen_1",
                     package='motor_control',
                     executable='set_point',
                     emulate_tty=True,
                     output='screen',
                     namespace="group1"
                     )

    control_node_1 = Node(name="control_node_1",
                          package='motor_control',
                          executable='controlador',
                          emulate_tty=True,
                          output='screen',
                          namespace="group1",
                          parameters=[{'kp': 2.0}],
                          )

    motor_node_2 = Node(name="motor_sys_2",
                        package='motor_control',
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

    sp_node_2 = Node(name="sp_gen_2",
                     package='motor_control',
                     executable='set_point',
                     emulate_tty=True,
                     output='screen',
                     namespace="group2"
                     )

    control_node_2 = Node(name="control_node_2",
                          package='motor_control',
                          executable='controlador',
                          emulate_tty=True,
                          output='screen',
                          namespace="group2",
                          parameters=[{'kp': 2.0}],
                          )

    motor_node_3 = Node(name="motor_sys_3",
                        package='motor_control',
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

    sp_node_3 = Node(name="sp_gen_3",
                     package='motor_control',
                     executable='set_point',
                     emulate_tty=True,
                     output='screen',
                     namespace="group3"
                     )

    control_node_3 = Node(name="control_node_3",
                          package='motor_control',
                          executable='controlador',
                          emulate_tty=True,
                          output='screen',
                          namespace="group3",
                          parameters=[{'kp': 2.0}],
                          )
    server_node_1 = Node(name="server_node_1",
                          package='motor_control',
                          executable='server',
                          emulate_tty=True,
                          output='screen',
                          namespace="group1",
                        )
    server_node_2 = Node(name="server_node_2",
                          package='motor_control',
                          executable='server',
                          emulate_tty=True,
                          output='screen',
                          namespace="group2",
                        )
    server_node_3 = Node(name="server_node_3",
                          package='motor_control',
                          executable='server',
                          emulate_tty=True,
                          output='screen',
                          namespace="group3",
                        )

    l_d = LaunchDescription([
        motor_node_1, sp_node_1, control_node_1,
        motor_node_2, sp_node_2, control_node_2,
        motor_node_3, sp_node_3, control_node_3, 
        server_node_1, server_node_2, server_node_3,
    ])
  
    return l_d
