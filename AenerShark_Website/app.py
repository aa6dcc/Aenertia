import os
import subprocess
import threading
from pathlib import Path

import serial
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.client as mqtt
import logging

# --- Configuration ---
MQTT_BROKER    = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT      = int(os.getenv("MQTT_PORT", 1883))
SERIAL_PORT    = os.getenv("SERIAL_PORT", "/dev/ttyUSB0")
BAUDRATE       = int(os.getenv("BAUDRATE", 115200))
SNAPSHOT_SCRIPT= Path(__file__).parent / "capture_image.py"
FLASH_LED_SCRIPT= Path(__file__).parent / "flash_led.py"
PID_LOG        = Path(__file__).parent / "pid_log.txt"

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# --- FastAPI App ---
app = FastAPI(
    title="Aenertia Robot Control API",
    description="REST & MQTT interface for Raspberry Pi + ESP32 robot"
)

# --- In‐memory state / connections ---
pid_values = {"inner": [], "outer": []}
serial_conn = None
mqtt_client = mqtt.Client()

# --- Serial Init ---
def init_serial():
    global serial_conn
    try:
        serial_conn = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        logger.info(f"Serial connected on {SERIAL_PORT} @ {BAUDRATE}")
    except Exception as e:
        serial_conn = None
        logger.warning(f"Serial connection failed: {e}")

# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, rc):
    logger.info(f"✅ MQTT connected (rc={rc})")
    for topic in ("robot/pid", "robot/led", "robot/serial"):
        client.subscribe(topic)
        logger.info(f"Subscribed to {topic}")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload = msg.payload.decode(errors="ignore")
    logger.debug(f"MQTT msg on {topic}: {payload}")

    if topic == "robot/pid":
        # format: "<inner|outer>:P,D,I,Set"
        try:
            loop_key, values = payload.split(":", 1)
            parts = values.split(",")
            if loop_key in pid_values:
                pid_values[loop_key].append(parts)
                with PID_LOG.open("a") as f:
                    f.write(f"{loop_key}: {values}\n")
                logger.info(f"Logged PID {loop_key}: {parts}")
        except Exception as e:
            logger.error(f"Malformed PID payload: {payload} ({e})")

    elif topic == "robot/led" and payload.strip().lower() == "flash":
        run_script(FLASH_LED_SCRIPT, sudo=True)

    elif topic == "robot/serial" and serial_conn:
        serial_conn.write((payload + "\n").encode())
        logger.info(f"Serial → ESP32: {payload}")

def run_script(script_path: Path, sudo: bool=False):
    cmd = (["sudo", "python3", str(script_path)] if sudo
           else ["python3", str(script_path)])
    try:
        subprocess.run(cmd, check=True)
        logger.info(f"Executed script: {' '.join(cmd)}")
    except subprocess.CalledProcessError as e:
        logger.error(f"Script error ({script_path}): {e}")

def start_mqtt_loop():
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    threading.Thread(target=mqtt_client.loop_forever, daemon=True).start()

# --- FastAPI Startup Event ---
@app.on_event("startup")
def startup_event():
    init_serial()
    start_mqtt_loop()

# --- API Routes ---

@app.get("/pid-values", summary="Get all PID tuning values")
def get_pid_values():
    return JSONResponse(content=pid_values)

@app.get("/snapshot", summary="Capture and return a camera snapshot")
def snapshot():
    if not SNAPSHOT_SCRIPT.exists():
        raise HTTPException(500, "Snapshot script not found")
    run_script(SNAPSHOT_SCRIPT)
    img = Path("snapshot.jpg")
    if img.exists():
        return FileResponse(str(img))
    raise HTTPException(500, "Failed to generate snapshot")

@app.get("/flash-led", summary="Flash the onboard LED")
def flash_led():
    if not FLASH_LED_SCRIPT.exists():
        raise HTTPException(500, "Flash-LED script not found")
    run_script(FLASH_LED_SCRIPT, sudo=True)
    return {"message": "LED flashed successfully"}

@app.get("/send-command", summary="Send raw command to ESP32 via serial")
def send_command(cmd: str):
    if not serial_conn:
        raise HTTPException(503, "Serial connection unavailable")
    serial_conn.write((cmd + "\n").encode())
    return {"message": f"Sent: {cmd}"}

# --- Static File Mounts (after routes!) ---
app.mount("/assets", StaticFiles(directory="assets"), name="assets")
app.mount("/",       StaticFiles(directory="static", html=True), name="static")
