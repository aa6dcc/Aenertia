import paho.mqtt.client as mqtt
import serial
import threading # Threading is a library that allows us to run other tasks that the current one in the background
from time import wait

#Serial configf
SERIAL_PORT = "/dev/ttyUSB0"
baud_rate = 115200

cv_enabled = False
ser = None


def send_2_esp(command):
    print(f"Sending to esp: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())


def fake_cv_loop(): # This loop will go until cv_disabled is called. Hozever because we are stuck in the loop we cannot check if the mqtt published disabled
    while cv_enabled:
        print("LED is on")
        send_2_esp("Led_ON")
        wait(2)
        print("LED is off")
        send_2_esp("LED_OFF")


def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code", rc)
    client.subscribe("cv/command")


def on_message(client, userdata, msg):
    global cv_enabled

    payload = msg.payload
    print(f"Topic:{msg.topic} ; Command: {payload}")

    if payload == "enable cv":
        if not cv_enabled:
            cv_enabled = True
            threading.Thread(target=fake_cv_loop, daemon=True).start() #To fix this we use threading. Threading isolates the code we target and procceeds zith the rest of the code.
    elif payload == "disable cv":
        cv_enabled = False
        send_2_esp("LED_OFF")


def main():
    global ser
    ser = serial.Serial(SERIAL_PORT, baud_rate, timeout=1)

    client = mqtt.Client() # Creat a client object from MQTT
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
