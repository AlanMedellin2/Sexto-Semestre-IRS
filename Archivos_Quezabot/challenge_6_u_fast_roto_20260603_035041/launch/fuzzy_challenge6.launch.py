from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ── Detector de línea (sin cambios, igual que siempre) ────────────
        Node(
            package='challenge_6_u_fast',
            executable='line_detector.py',
            name='line_detector',
            output='screen'
        ),

        # ── YOLO + área de señal (reemplaza yolo_decision_node.py) ────────
        Node(
            package='challenge_6_u_fast',
            executable='yolo_double_detector_fuzzy.py',
            name='yolo_decision_node',
            output='screen'
        ),

        # ── Controlador difuso (reemplaza line_follower.py) ───────────────
        Node(
            package='challenge_6_u_fast',
            executable='fuzzy_follower.py',
            name='line_follower_pid',
            output='screen'
        ),

    ])
