import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)

def on_message(client, userdata, msg):
    data = msg.payload.decode()
    print("Received:", data)

    GPIO.output(17, data == "LED1")
    GPIO.output(27, data == "LED2")

    if data == "OFF":
        GPIO.output(17, False)
        GPIO.output(27, False)

client = mqtt.Client()
client.connect("localhost", 1883)
client.subscribe("led/control")
client.on_message = on_message

client.loop_forever()
