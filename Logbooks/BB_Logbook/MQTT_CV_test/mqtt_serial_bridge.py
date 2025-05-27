import paho.mqtt.client as mqtt
import serial
import threading # Threading is a library that allows us to run other tasks that the current one in the background
from time import sleep

import numpy as np
import cv2

#Serial configf
SERIAL_PORT = "/dev/ttyUSB0"
baud_rate = 115200

cv_enabled = False
ser = None
mode = "manual" #Global var mode initialized to Manual


def send_2_esp(command):
    print(f"Sending to esp: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())


def fake_cv_loop(): # This loop will go until cv_disabled is called. Hozever because we are stuck in the loop we cannot check if the mqtt published disabled
    while cv_enabled:
        print("LED is on")
        send_2_esp("LED_ON")
        sleep(2)
        print("LED is off")
        send_2_esp("LED_OFF")
        sleep(2)

def cv_loop():
    pass



def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/cv")
    client.subscribe("robot/manual/command")



def on_message(client, userdata, msg):
    global cv_enabled
    global mode
    payload = msg.payload.decode()
    topic = msg.topic.decode()
    print(f"Topic:{msg.topic} ; Command: {payload}")

    # We start by checking if the mode was changed
    if topic == "robot/mode":
        mode = payload

    # Manual mode code
    if mode == "manual":
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
        elif payload == "up-right":
            send_2_esp("backwardANDright")
        elif payload == "up-left":
            send_2_esp("backwardANDleft")

    # Autnomous mode code
    elif mode == "autonomous":    
        if topic == "robot/cv":
            if payload == "enable cv":
                if not cv_enabled:
                    cv_enabled = True
                    print("Cv enabled")
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
