#!/usr/bin/env python3

import time
import threading
import cv2
import numpy as np

from flask import Flask, Response, jsonify

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Float32, String, Bool
from cv_bridge import CvBridge


app = Flask(__name__)

STREAM_WIDTH  = 320
STREAM_HEIGHT = 240
JPEG_QUALITY  = 60

latest_raw       = None
latest_image     = None
latest_yolo      = None
latest_error     = 0
latest_yolo_cmd  = "none"
latest_sign_area = 0.0
latest_color     = 0.0
latest_finish    = False
latest_inter     = False

lock = threading.Lock()


class CameraTopicWeb(Node):
    def __init__(self):
        super().__init__("camera_topic_web_server")
        self.bridge = CvBridge()

        self.create_subscription(Image,   "/camera/raw",        self.raw_cb,   10)
        self.create_subscription(Image,   "/camera/image",      self.img_cb,   10)
        self.create_subscription(Image,   "/yolo/debug",        self.yolo_cb,  10)
        self.create_subscription(Int32,   "/line_error",        self.err_cb,   10)
        self.create_subscription(String,  "/yolo/command",      self.cmd_cb,   10)
        self.create_subscription(Float32, "/yolo/sign_area",    self.area_cb,  10)
        self.create_subscription(Float32, "/color",             self.color_cb, 10)
        self.create_subscription(Bool,    "/finish_line",       self.fin_cb,   10)
        self.create_subscription(Bool,    "/intersection_line", self.inter_cb, 10)

        self.get_logger().info("camera_web_full: todos los tópicos listos")

    def _to_frame(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def raw_cb(self, msg):
        global latest_raw
        try:
            f = self._to_frame(msg)
            with lock: latest_raw = f.copy()
        except Exception as e:
            self.get_logger().error(f"raw_cb: {e}")

    def img_cb(self, msg):
        global latest_image
        try:
            f = self._to_frame(msg)
            with lock: latest_image = f.copy()
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def yolo_cb(self, msg):
        global latest_yolo
        try:
            f = self._to_frame(msg)
            with lock: latest_yolo = f.copy()
        except Exception as e:
            self.get_logger().error(f"yolo_cb: {e}")

    def err_cb(self, msg):
        global latest_error
        with lock: latest_error = int(msg.data)

    def cmd_cb(self, msg):
        global latest_yolo_cmd
        with lock: latest_yolo_cmd = str(msg.data)

    def area_cb(self, msg):
        global latest_sign_area
        with lock: latest_sign_area = float(msg.data)

    def color_cb(self, msg):
        global latest_color
        with lock: latest_color = float(msg.data)

    def fin_cb(self, msg):
        global latest_finish
        with lock: latest_finish = bool(msg.data)

    def inter_cb(self, msg):
        global latest_inter
        with lock: latest_inter = bool(msg.data)


def _blank(text="Sin señal"):
    f = np.zeros((STREAM_HEIGHT, STREAM_WIDTH, 3), dtype=np.uint8)
    cv2.putText(f, text, (20, STREAM_HEIGHT//2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1)
    return f


def _stream(get_frame_fn):
    while True:
        frame = get_frame_fn()
        frame = cv2.resize(frame, (STREAM_WIDTH, STREAM_HEIGHT))
        ok, buf = cv2.imencode(".jpg", frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")
        time.sleep(0.05)


def get_raw():
    with lock:
        f = latest_raw
    return f.copy() if f is not None else _blank("/camera/raw")


def get_image():
    with lock:
        f = latest_image
    if f is not None:
        frame = f.copy()
        with lock:
            err = latest_error
        h, w = frame.shape[:2]
        cx = w // 2
        tx = max(0, min(w-1, cx + err))
        ty = int(h * 0.3)
        cv2.line(frame, (cx, h), (cx, 0), (255, 0, 0), 1)
        cv2.line(frame, (cx, h-5), (tx, ty), (0, 255, 255), 2)
        cv2.circle(frame, (tx, ty), 5, (0, 255, 255), -1)
        cv2.putText(frame, f"E:{err}", (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        return frame
    return _blank("/camera/image")


def get_yolo():
    with lock:
        f = latest_yolo
    return f.copy() if f is not None else _blank("/yolo/debug")


@app.route("/video_feed")
def video_feed():
    return Response(_stream(get_raw),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video_raw")
def video_raw():
    return Response(_stream(get_raw),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video_image")
def video_image():
    return Response(_stream(get_image),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/video_yolo")
def video_yolo():
    return Response(_stream(get_yolo),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/line_error")
def line_error():
    with lock: e = latest_error
    return jsonify({"line_error": e})

@app.route("/yolo_command")
def yolo_command():
    with lock: c = latest_yolo_cmd
    return jsonify({"yolo_command": c})

@app.route("/yolo_sign_area")
def yolo_sign_area():
    with lock: a = latest_sign_area
    return jsonify({"yolo_sign_area": a})

@app.route("/color")
def color():
    with lock: c = latest_color
    return jsonify({"color": c})

@app.route("/finish_line")
def finish_line():
    with lock: f = latest_finish
    return jsonify({"finish_line": f})

@app.route("/intersection_line")
def intersection_line():
    with lock: i = latest_inter
    return jsonify({"intersection_line": i})

@app.route("/")
def index():
    return """<html><body style="background:#070b14;color:white;font-family:monospace;padding:20px">
    <h2>PuzzleBot Camera Server</h2>
    <div style="display:flex;gap:12px;flex-wrap:wrap">
      <div><p>/camera/raw</p><img src="/video_raw" style="width:320px;border-radius:8px"></div>
      <div><p>/camera/image</p><img src="/video_image" style="width:320px;border-radius:8px"></div>
      <div><p>/yolo/debug</p><img src="/video_yolo" style="width:320px;border-radius:8px"></div>
    </div>
    </body></html>"""


def ros_spin():
    rclpy.init()
    node = CameraTopicWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print("PuzzleBot camera server — puerto 5000")
    threading.Thread(target=ros_spin, daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
