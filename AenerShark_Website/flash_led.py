import RPi.GPIO as GPIO
import time

LED_PIN = 4  # GPIO 4 (pin 7)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

# Flash the LED
GPIO.output(LED_PIN, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(LED_PIN, GPIO.LOW)

GPIO.cleanup()
