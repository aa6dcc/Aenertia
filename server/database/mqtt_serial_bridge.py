import paho.mqtt.client as mqtt
import serial
import threading
from time import sleep
from Advanced_CV.pose_detection import pose_detection, send_frame
from flask import Flask, Response, send_from_directory
import global_var as gv
import json
import os
import cv2

from server.database.dynamodb_telemetry_logger import start_telemetry_logger, set_mode, set_follow

SERIAL_PORT = [f"/dev/ttyUSB{i}" for i in range(20)]
baud_rate = 115200
ser = None
follow_mode = False
mode = "manual"

# FLASK APP
app = Flask(__name__, static_folder='Placeholder_UI/static', static_url_path='')

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/video_feed')
def video_feed():
    return Response(
        send_frame(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

def start_web():
    app.run(host='0.0.0.0', port=8001, threaded=True)

def send_2_esp(command):
    print(f"Sending to ESP: {command}")
    if ser and ser.is_open:
        ser.write((command + "\n").encode())

def follow_me():
    while follow_mode:
        if gv.HumanDetected:
            offset = gv.offset
            if abs(offset) < 200:
                send_2_esp("forward")
            elif 200 <= offset < 700:
                send_2_esp("forwardANDright")
            elif -700 < offset <= -200:
                send_2_esp("forwardANDleft")
            elif offset >= 700:
                send_2_esp("right")
            elif offset <= -700:
                send_2_esp("left")
            else:
                send_2_esp("stop")
        else:
            send_2_esp("stop")
        sleep(0.5)

def gotoKeyLocation():
    print("ROS2 placeholder function: gotoKeyLocation()")

def esp_read():
    while True:
        if ser and ser.in_waiting > 0:
            try:
                incoming = ser.readline().decode().strip()
                print(incoming)
                if incoming.startswith("POS:"):
                    data = json.loads(incoming[4:].strip())
                    gv.robot_x = data.get("x", 0.0)
                    gv.robot_y = data.get("y", 0.0)
            except UnicodeDecodeError:
                print("ESP decode error")
            except json.JSONDecodeError:
                print("ESP JSON parse error")

def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code", rc)
    client.subscribe("robot/mode")
    client.subscribe("robot/auto")
    client.subscribe("robot/manual/command")
    threading.Thread(target=pose_detection, daemon=True).start()
    threading.Thread(target=esp_read, daemon=True).start()

def on_message(client, userdata, msg):
    global mode, follow_mode
    payload = msg.payload.decode()
    topic = msg.topic
    print(f"Topic:{topic} ; Command: {payload}")

    if topic == "robot/mode":
        mode = payload
        set_mode(mode)

    if topic == "robot/auto" and payload == "show_keys":
        from server.database.dynamodb import get_all_key_locations
        keys = get_all_key_locations()
        key_names = [k['name'] for k in keys]
        client.publish("robot/keys", json.dumps(key_names))
        return

    if mode != "autonomous" or payload == "return":
        follow_mode = False
        set_follow(False)
        print("Follow mode OFF")

    if mode == "manual":
        command_map = {
            "up": "forward", "down": "backward", "left": "left", "right": "right",
            "up-right": "forwardANDright", "up-left": "forwardANDleft",
            "down-right": "backwardANDright", "down-left": "backwardANDleft",
            "stop": "stop"
        }
        if payload in command_map:
            send_2_esp(command_map[payload])

    elif mode == "autonomous" and topic == "robot/auto":
        if payload == "follow":
            follow_mode = True
            set_follow(True)
            threading.Thread(target=follow_me, daemon=True).start()
        elif payload == "return":
            gotoKeyLocation()
        elif payload == "stop":
            send_2_esp("stop")

def main():
    global ser
    for port in SERIAL_PORT:
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            print(f"Serial connection start using port: {port}")
            break
        except (FileNotFoundError, serial.SerialException):
            print(f"Failed to connect to port: {port}")

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)

    start_telemetry_logger()

    client.loop_forever()

if __name__ == "__main__":
    threading.Thread(target=start_web, daemon=True).start()
    main()
