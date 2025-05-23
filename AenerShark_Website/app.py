# ===============================
# 📄 app.py (FastAPI backend for robot)
# ===============================

from fastapi import FastAPI, BackgroundTasks
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import subprocess
import serial
import os
import threading
import time
import paho.mqtt.client as mqtt

app = FastAPI()

# 1) assets first
app.mount("/assets", StaticFiles(directory="assets"), name="assets")
# 2) then the catch-all
app.mount("/", StaticFiles(directory="static", html=True), name="static")



# PID storage
pid_values = {
    'inner': [],
    'outer': []
}

# Serial connection to ESP32 (adjust port and baudrate)
try:
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
except Exception as e:
    print("Serial connection failed:", e)
    ser = None

# MQTT setup
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883

def on_connect(client, userdata, flags, rc):
    print(f"✅ MQTT connected with result code {rc}")
    client.subscribe("robot/pid")
    client.subscribe("robot/led")
    client.subscribe("robot/serial")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()

    if topic == "robot/pid":
        if ':' in payload:
            loop, values = payload.split(':')
            parts = values.split(',')
            print(f"MQTT PID for {loop}: {parts}")
            if loop in pid_values:
                pid_values[loop].append(parts)
                with open("pid_log.txt", "a") as f:
                    f.write(f"{loop}: {values}\n")

    elif topic == "robot/led" and payload == "flash":
        try:
            subprocess.run(["sudo", "python3", "flash_led.py"], check=True)
            print("💡 LED flashed")
        except subprocess.CalledProcessError as e:
            print("LED error:", e)

    elif topic == "robot/serial" and ser:
        ser.write((payload + "\n").encode())
        print(f"➡️ Sent to ESP32: {payload}")

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

def mqtt_loop():
    mqtt_client.loop_forever()

# Start MQTT loop in background
threading.Thread(target=mqtt_loop, daemon=True).start()

@app.get("/pid-values")
def get_pid_values():
    return JSONResponse(content=pid_values)

@app.get("/snapshot")
def get_snapshot():
    try:
        subprocess.run(["python3", "capture_image.py"], check=True)
        return FileResponse("snapshot.jpg")
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@app.get("/flash-led")
def flash_led():
    try:
        subprocess.run(["sudo", "python3", "flash_led.py"], check=True)
        return {"message": "LED flashed!"}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

@app.get("/send-command")
def send_serial_command(cmd: str):
    if not ser:
        return JSONResponse(status_code=500, content={"error": "Serial not connected"})
    ser.write((cmd + "\n").encode())
    return {"message": f"Sent to ESP32: {cmd}"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=5000, reload=True)
