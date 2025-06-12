#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
import time
import threading
import cv2

# Shim for Picamera2 → OpenCV/V4L2
class Picamera2:
    def __init__(self):
        # open /dev/video0 via V4L2
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open /dev/video0 – check camera & permissions")
    def configure(self, *args, **kwargs):
        # no-op: you can set resolution here if you like, or depend on cap.set() below
        return None
    def start(self):
        # no-op for VideoCapture
        return None
    def capture_array(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to grab frame from camera")
        return frame
    def stop(self):
        self.cap.release()


# Tasks-API imports (unchanged)
import mediapipe as mp
from mediapipe.tasks.python.core.base_options import BaseOptions
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import (
    PoseLandmarker,
    PoseLandmarkerOptions,
)

import global_var as gv


latest_frame = None
frame_lock = threading.Lock()


def pose_detection():
    global latest_frame

    # 1) Load your .task bundle
    model_path = os.path.expanduser("~/pose_landmarker_lite.task")
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")
    base_options = BaseOptions(model_asset_path=model_path)

    options = PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,        # synchronous VIDEO mode
        min_pose_detection_confidence=0.5,
    )
    landmarker = PoseLandmarker.create_from_options(options)

    # 2) “Start” the camera via our shim
    n = 2  # unused by shim but preserved for API compatibility
    cam = Picamera2()
    # if you want to change resolution, do:
    cam.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640 * n)
    cam.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480 * n)
    cam.configure(None)   # no-op
    cam.start()           # no-op
    time.sleep(1)         # allow auto-exposure / AWB to settle

    mp_image_format = mp.ImageFormat.SRGB

    while True:
        # 3) Capture & wrap frame
        frame_bgr = cam.capture_array()
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(
            image_format=mp_image_format,
            data=frame_rgb
        )

        timestamp_ms = int(time.time() * 1000)
        try:
            result = landmarker.detect_for_video(mp_image, timestamp_ms)
        except Exception as e:
            print("⚠️ Inference error:", e)
            continue

        # 5) Pull out only hips/knees/ankles from the FIRST pose detection
        lower_idxs = [11, 12, 13, 14, 15, 16]
        pts = []
        h, w = frame_bgr.shape[:2]

        gv.offset = 0
        if result.pose_landmarks:
            gv.HumanDetected = True
            first_pose = result.pose_landmarks[0]
            for idx in lower_idxs:
                kp = first_pose[idx]
                if 0 <= kp.x <= 1 and 0 <= kp.y <= 1:
                    x_px = int(kp.x * w)
                    y_px = int(kp.y * h)
                    pts.append((x_px, y_px))
        else:
            gv.HumanDetected = False

        # 6) Draw bounding box & compute offset
        if pts:
            xs, ys = zip(*pts)
            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            gv.offset = ((x1 + x2) // 2) - (w // 2)

        with frame_lock:
            latest_frame = frame_bgr.copy()

    # never reaches here, but for completeness:
    # landmarker.close()
    # cam.stop()
    # cv2.destroyAllWindows()


def send_frame():
    """Generator for Flask’s multipart JPEG stream."""
    global latest_frame

    while True:
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            time.sleep(0.05)
            continue

        ok, buf = cv2.imencode('.jpg', frame)
        if not ok:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buf.tobytes() +
            b'\r\n'
        )


if __name__ == "__main__":
    pose_detection()
