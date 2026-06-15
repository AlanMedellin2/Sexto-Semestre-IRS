#!/usr/bin/env python3

import time
import threading
import cv2
from flask import Flask, Response

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


app = Flask(__name__)

latest_frame = None
frame_lock = threading.Lock()


class RosImageWebStream(Node):
    def __init__(self):
        super().__init__('ros_image_web_stream')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            1
        )

        self.get_logger().info("Stream web ligero leyendo /camera/image")

    def image_callback(self, msg):
        global latest_frame

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Reducir desde callback para guardar menos memoria
        frame = cv2.resize(frame, (320, 180))

        with frame_lock:
            latest_frame = frame


def generate():
    global latest_frame

    while True:
        time.sleep(0.5)  # 2 FPS máximo

        with frame_lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)

        if not ret:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n'
        )


@app.route('/')
def index():
    return """
    <html>
        <head>
            <title>Challenge 5 Debug</title>
        </head>
        <body style="background:#111; color:white; text-align:center;">
            <h2>Challenge 5 - Debug ligero</h2>
            <p>Stream 2 FPS para no saturar CPU</p>
            <img src="/video_feed" width="640">
        </body>
    </html>
    """


@app.route('/video_feed')
def video_feed():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def ros_thread():
    rclpy.init()
    node = RosImageWebStream()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()

    app.run(host='0.0.0.0', port=5001, threaded=True)

