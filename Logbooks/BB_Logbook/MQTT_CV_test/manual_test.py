import paho.mqtt.client as mqtt
import serial
import threading
from time import sleep
import json
import global_var as gv

# Serial config (check common USB ports)
SERIAL_PORT = [
    "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4",
    "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7", "/dev/ttyUSB8", "/dev/ttyUSB9",
    "/dev/ttyUSB10", "/dev/ttyUSB11", "/dev/ttyUSB12", "/dev/ttyUSB13", "/dev/ttyUSB14",
    "/dev/ttyUSB15", "/dev/ttyUSB16", "/dev/ttyUSB17", "/dev/ttyUSB18", "/dev/ttyUSB19"
]

baud_rate = 115200

# Global variables
ser = None
mode = "manual"

def send_2_esp(command):
    print(f"Sending to ESP: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())
        print(command)

def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/manual/command")

def on_message(client, userdata, msg):
    global mode

    payload = msg.payload.decode()
    topic = msg.topic
    print(f"Topic: {topic} | Command: {payload}")

    if topic == "robot/mode":
        mode = payload

    if mode == "manual" and topic == "robot/manual/command":
        if payload == "up":
            send_2_esp("forward")
        elif payload == "down":
            send_2_esp("backward")
        elif payload == "right":
            send_2_esp("right")
        elif payload == "left":
            send_2_esp("left")
        elif payload == "up-right":
            send_2_esp("forwardANDright")
        elif payload == "up-left":
            send_2_esp("forwardANDleft")
        elif payload == "down-right":
            send_2_esp("backwardANDright")
        elif payload == "down-left":
            send_2_esp("backwardANDleft")
        else:
            send_2_esp("stop")

def main():
    global ser

    for port in SERIAL_PORT:
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Serial connection started on port: {port}")
            break
        except (FileNotFoundError, serial.SerialException):
            print(f"Failed to connect on port: {port}")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("172.20.10.10", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
