#!/usr/bin/env python3
import os
import subprocess
import logging
import serial
import paho.mqtt.client as mqtt

# --- Config ---
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", 1883))
SERIAL_PORT = os.getenv("SERIAL_PORT", "/dev/ttyUSB0")
BAUDRATE    = int(os.getenv("BAUDRATE", 115200))

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger()

# --- Serial init ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    logger.info(f"Serial on {SERIAL_PORT}@{BAUDRATE}")
except Exception as e:
    ser = None
    logger.warning(f"Serial unavailable: {e}")

# --- MQTT callbacks ---
def on_connect(client, userdata, flags, rc):
    logger.info(f"MQTT connected (rc={rc})")
    for t in ("robot/serial", "led/control", "cv/control", "autonomous", "robot/pid", "mode/set"):
        client.subscribe(t)
        logger.info(f"Subscribed to {t}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode()
    logger.info(f"→ {topic} = {payload}")

    if topic == "led/control" and payload == "FLASH":
        # you said to forget flash_led; so just log here
        logger.info("FLASH command received")

    elif topic == "robot/serial" and ser:
        ser.write((payload + "\n").encode())

    elif topic == "cv/control":
        logger.info(f"CV control: {payload}")

    elif topic == "autonomous":
        logger.info(f"Autonomous cmd: {payload}")

    elif topic == "robot/pid":
        logger.info(f"PID tuning: {payload}")

    elif topic == "mode/set":
        logger.info(f"Mode set to {payload}")

# --- Start MQTT client ---
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever()
