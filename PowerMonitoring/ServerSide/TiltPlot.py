import serial
import json
import os
import csv

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'      # Change to your actual serial port
BAUD_RATE = 115200                # Match the baud rate with your device
CSV_FILE = 'tilt_log.csv'
# ======================


def initialize_csv_file():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["TILT", "TIME"])

def append_to_csv(TILT, TIME):
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([TILT,TIME])

def main():
    initialize_csv_file()
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print("Listening for serial input...")
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                if line[0:5] == "TILT:":
                    line = line.split()[1]
                    data = json.loads(line)
                    TILT = data.get("tilt")
                    TIME = data.get("time")

                    if TILT is not None and TIME is not None:
                        append_to_csv(TILT, TIME)
                        print(f"Logged: {TILT}, {TIME}")
                    else:
                        print("Missing one or more required keys in JSON.")

            except json.JSONDecodeError:
                print("Invalid JSON received.")
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    main()

