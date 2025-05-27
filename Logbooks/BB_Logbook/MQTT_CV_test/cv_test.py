import numpy as np
import cv2
from picamera2 import Picamera2
from time import sleep

cv_enabled = True

cam = Picamera2()
lower_body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_lowerbody.xml")
if lower_body_cascade.empty():
    print("empty") 

cam.configure(cam.create_video_configuration(main={"size":(640*3, 480*3), "format": "RGB888"}))
cam.start()

sleep(1)

while cv_enabled:
        
    frame = cam.capture_array()
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lower_bodies = lower_body_cascade.detectMultiScale(grey, 1.1, 2)
    
    for(x, y, h, w) in lower_bodies:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    #resized = cv2.resize(frame, (2080, 1500))

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    sleep(0.001)


cv2.destroyAllWindows()