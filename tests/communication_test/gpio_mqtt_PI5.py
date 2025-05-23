import paho.mqtt.client as mqtt
from gpiozero import LED

# Set up GPIO pins
led1 = LED(17)
led2 = LED(27)

# Define MQTT behavior
def on_message(client, userdata, msg):
    data = msg.payload.decode()
    print("Received:", data)

    # Turn off both LEDs first
    led1.off()
    led2.off()

    # Turn on one if instructed
    if data == "LED1":
        led1.on()
    elif data == "LED2":
        led2.on()

# Set up MQTT
client = mqtt.Client()
client.connect("localhost", 1883)
client.subscribe("led/control")
client.on_message = on_message

client.loop_forever()
