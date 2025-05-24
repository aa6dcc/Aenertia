from picamera2 import Picamera2
import cv2
from time import sleep

picam2 = Picamera2()

#Config frame size 
config = picam2.create_still_configuration(main={"size": (3280, 2464)})
picam2.configure(config)

#Start Camera
picam2.start()
sleep(1)

#Display Video
while(True):
    frame = picam2.capture_array()
    resized = cv2.resize(frame, (1280, 960))

    cv2.imshow("Camera", resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
