import threading
import time
import json
from server.database.dynamodb import (
    update_robot_state,
    log_location,
    find_nearest_key_location
)
from server.database.utils import get_battery_level
import global_var as gv

mode = "manual"
follow_mode = False

def set_mode(m):
    global mode
    mode = m

def set_follow(f):
    global follow_mode
    follow_mode = f

def send_telemetry_to_dynamodb():
    device_id = "esp32-001"
    while True:
        try:
            x = getattr(gv, "robot_x", 0.0)
            y = getattr(gv, "robot_y", 0.0)
            battery = get_battery_level()
            current_mode = mode
            current_status = "MOVING" if follow_mode else "IDLE"
            current_key_loc = find_nearest_key_location(x, y)

            update_robot_state(device_id, x, y, battery, current_mode, current_status, current_key_loc)
            log_location(device_id, x, y)

            print("[DynamoDB] Telemetry sent:", {
                "x": x, "y": y, "battery": battery,
                "mode": current_mode, "status": current_status,
                "key": current_key_loc
            })

        except Exception as e:
            print("[DynamoDB] Error:", e)

        time.sleep(5)

def start_telemetry_logger():
    threading.Thread(target=send_telemetry_to_dynamodb, daemon=True).start()
