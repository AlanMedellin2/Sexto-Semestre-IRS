#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from collections import deque
from flask import Flask, Response

app = Flask(__name__)

cap = cv.VideoCapture('/dev/video1', cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 15)

Area_min = 200
Area_max = 50000
H_history = deque(maxlen=10)


def get_debug_frame():
    ret, originalFr = cap.read()

    if not ret:
        return None

    h, w = originalFr.shape[:2]

    roi = originalFr[int(h * 0.6):h, :]
    roi_h, roi_w = roi.shape[:2]

    debug = roi.copy()

    H_roi = roi[int(roi_h * 0.60):roi_h, int(roi_w * 0.30):int(roi_w * 0.70)]
    hsv_roi = cv.cvtColor(H_roi, cv.COLOR_BGR2HSV)

    H_channel = hsv_roi[:, :, 2]
    H_mean = np.mean(H_channel)

    H_history.append(H_mean)
    H_mean_smooth = np.mean(H_history)

    gris_image = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
    blurred = cv.GaussianBlur(gris_image, (5, 5), 0)

    if H_mean_smooth < 110.00:
        cutting = 100
    elif H_mean_smooth < 135.0:
        cutting = 140
    elif H_mean_smooth < 150.0:
        cutting = 140
    elif H_mean_smooth < 155.0:
        cutting = 115
    elif H_mean_smooth < 165.0:
        cutting = 120
    elif H_mean_smooth < 170.0:
        cutting = 125
    elif H_mean_smooth < 190.0:
        cutting = 130
    elif H_mean_smooth < 200.0:
        cutting = 140
    else:
        cutting = 160

    _, binary = cv.threshold(
        blurred,
        cutting,
        255,
        cv.THRESH_BINARY_INV
    )

    top_width = int(roi_w * 0.6)
    top_y = int(roi_h * 0.3)

    trapezoid = np.array([[
        ((roi_w - top_width) // 2, top_y),
        ((roi_w + top_width) // 2, top_y),
        (roi_w, roi_h),
        (0, roi_h)
    ]], dtype=np.int32)

    mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
    cv.fillPoly(mask, trapezoid, 255)

    binary_masked = cv.bitwise_and(binary, binary, mask=mask)

    kernel = np.ones((3, 3), np.uint8)
    morph = cv.erode(binary_masked, kernel, iterations=1)
    morph = cv.dilate(morph, kernel, iterations=1)

    num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(
        morph,
        connectivity=8
    )

    ref_x = roi_w // 2
    candidatos = []

    for i in range(1, num_labels):
        x, y, bw, bh, area = stats[i]
        cx, cy = centroids[i]

        if Area_min <= area <= Area_max:
            candidatos.append((cx, cy, area, x, y, bw, bh))

    cv.polylines(debug, trapezoid, True, (255, 0, 255), 2)

    cv.line(debug, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 2)
    cv.line(debug, (ref_x - 50, 0), (ref_x - 50, roi_h), (255, 0, 0), 1)
    cv.line(debug, (ref_x + 50, 0), (ref_x + 50, roi_h), (255, 0, 0), 1)

    error_x = 0

    if len(candidatos) > 0:
        cx, cy, area, x, y, bw, bh = min(
            candidatos,
            key=lambda p: abs(p[0] - ref_x)
        )

        error_x = int(cx - ref_x)

        cv.rectangle(debug, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv.circle(debug, (int(cx), int(cy)), 6, (0, 255, 255), -1)

        cv.line(
            debug,
            (ref_x, roi_h),
            (int(cx), int(cy)),
            (0, 255, 255),
            2
        )

        cv.putText(
            debug,
            f"centroide: ({int(cx)}, {int(cy)})",
            (int(cx) + 10, int(cy)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1
        )
    else:
        cv.putText(
            debug,
            "Linea no detectada",
            (20, 35),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    cv.putText(
        debug,
        f"error: {error_x}",
        (20, 30),
        cv.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2
    )

    cv.putText(
        debug,
        f"brillo: {H_mean_smooth:.1f} cutting: {cutting}",
        (20, 60),
        cv.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2
    )

    return debug


def generate():
    while True:
        frame = get_debug_frame()

        if frame is None:
            continue

        ret, buffer = cv.imencode('.jpg', frame)

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
            <title>Line Detector Debug</title>
        </head>
        <body style="background:#111; color:white; text-align:center;">
            <h2>Challenge 5 - Line Debug</h2>
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


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
