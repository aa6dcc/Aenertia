import  serial
import time

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
time.sleep(2)

ser.write(b"LED_ON")
print("LED_ON")

time.sleep(5)

ser.write(b"LED_OFF")
print("LED_OFF")
