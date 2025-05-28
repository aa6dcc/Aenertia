# camera_stream.py

import cv2
from time import sleep

# 1. Open your default webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam")

# 2. Load the Haar cascade for lower-body detection
lower_body_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_lowerbody.xml"
)
if lower_body_cascade.empty():
    raise RuntimeError("Failed to load Haar cascade")

def gen_frames():
    """
    Generator that yields processed MJPEG frames:
    - Read from webcam
    - Convert to grayscale
    - Detect lower bodies
    - Draw blue rectangles
    - JPEG-encode and yield
    """
    while True:
        success, frame = cap.read()
        if not success:
            break

        # 3. Pre-process for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 4. Detect lower bodies
        bodies = lower_body_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=2)

        # 5. Draw rectangles on the color frame
        for (x, y, w, h) in bodies:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # 6. JPEG-encode the annotated frame
        ret, jpeg = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        frame_bytes = jpeg.tobytes()
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            frame_bytes +
            b'\r\n'
        )

        # control frame rate
        sleep(0.03)  # ~30 FPS

    cap.release()
