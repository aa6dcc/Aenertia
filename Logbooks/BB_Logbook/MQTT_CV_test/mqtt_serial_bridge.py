import paho.mqtt.client as mqtt
import serial
import threading # Threading is a library that allows us to run other tasks that the current one in the background
from time import sleep
#import app_video # runs the server to send back the video
from Advanced_CV.pose_detection import pose_detection

import numpy as np
import cv2

#Serial config (i included many ports just n case we somehow connect to an unexpected port number. It is very unlikely it goes above 1) 
SERIAL_PORT = [ "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2","/dev/ttyUSB3", "/dev/ttyUSB5", 
                "/dev/ttyUSB6", "/dev/ttyUSB7", "/dev/ttyUSB8","/dev/ttyUSB9", "/dev/ttyUSB10", 
                "/dev/ttyUSB11", "/dev/ttyUSB12", "/dev/ttyUSB13","/dev/ttyUSB14", "/dev/ttyUSB15", 
                "/dev/ttyUSB15", "/dev/ttyUSB16", "/dev/ttyUSB17","/dev/ttyUSB18", "/dev/ttyUSB19" ]

baud_rate = 115200

#global variables
cv_enabled = False
ser = None
mode = "manual" #Global var mode initialized to Manual
HumanDetected = False
offset = 0


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
    global offset
    pass
    '''
    return 
    '''

def GotoKeyLocation():
    print("ROS2 works ; Python stays silent")

def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/cv")
    client.subscribe("robot/manual/command")

    # Run the CV pose detection in the background
    threading.Thread(target=pose_detection, daemon=True).start() #To fix this we use threading. Threading isolates the code we target and procceeds zith the rest of the code.



def on_message(client, userdata, msg):
    #global cv_enabled
    global mode
    global HumanDetected
    global offset

    #MQTT input
    payload = msg.payload.decode()
    topic = msg.topic
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
        elif payload == "down-right":
            send_2_esp("backwardANDright")
        elif payload == "down-left":
            send_2_esp("backwardANDleft")
        else:
            send_2_esp("stop")

    # Autnomous mode code
    elif mode == "autonomous":  
        ''' PREVIOUS TEST: CURRENTLY IRRELEVANT
        if topic == "robot/cv":
            if payload == "enable cv":
                if not cv_enabled:
                    cv_enabled = True
                    print("Cv enabled")
                    threading.Thread(target=fake_cv_loop, daemon=True).start() #To fix this we use threading. Threading isolates the code we target and procceeds zith the rest of the code.
            elif payload == "disable cv":
                cv_enabled = False
                send_2_esp("LED_OFF")
        '''
        if topic == "robot/auto":
            # In follow mode the robot follors the person around
            if payload == "follow":
                if HumanDetected:
                    if abs(offset) < 100: 
                        send_2_esp("forward")
                    elif 100 <= offset < 700:
                        send_2_esp("forward and right")
                    elif -100 >= offset > -700:
                        send_2_esp("forzard and left")
                    elif offset >= 700:
                        send_2_esp("right")
                    elif offset <= -700:
                        send_2_esp("left")
                    else: 
                        send_2_esp("stop")
                else :
                    send_2_esp("stop")
            
            elif payload == "GotoKeyLocation":
                GotoKeyLocation()

            elif payload == "stop": #To be implemented later
                send_2_esp("stop")


def main():
    #Robot function
    #Telemetry loop
    global ser

    for port in SERIAL_PORT:
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Serial connection start using port: {port}")
            break
        except FileNotFoundError:
            print(f"failed to connect to port: {port}")

    client = mqtt.Client() # Creat a client object from MQTT
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()


if __name__ == "__main__":
    main()
