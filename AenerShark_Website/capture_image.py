# capture_image.py
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
ret, frame = cap.read()
cap.release()

if ret:
    cv2.imwrite("snapshot.jpg", frame)
    print("OK")
else:
    print("ERROR")

