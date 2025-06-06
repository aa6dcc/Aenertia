import serial
import json
import os
import csv

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'      # Change to your actual serial port
BAUD_RATE = 115200                # Match the baud rate with your device
CSV_FILE = 'power_log.csv'
# ======================

def initialize_csv_file():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Voltage", "5V", "Current_Motor", "Current_Board"])

def append_to_csv(voltage, voltage5, current_motor, current_board):
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([voltage, voltage5, current_motor, current_board])

def main():
    initialize_csv_file()
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        print("Listening for serial input...")
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                if line[0:3] == "PM:":
                    line = line.split()[1]
                    data = json.loads(line)
                    voltage = data.get("VB")
                    voltage5 = data.get("V5")
                    current_motor = data.get("CM")
                    current_board = data.get("CB")

                    if voltage is not None and voltage5 is not None and current_motor is not None and current_board is not None:
                        append_to_csv(voltage, voltage5, current_motor, current_board)
                        print(f"Logged: {voltage}, {voltage5}, {current_motor}, {current_board}")
                    else:
                        print("Missing one or more required keys in JSON.")

            except json.JSONDecodeError:
                print("Invalid JSON received.")
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    main()

