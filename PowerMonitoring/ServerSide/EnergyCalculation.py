import serial
import json
import os
import csv

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'      # Change to your actual serial port
BAUD_RATE = 115200                # Match the baud rate with your device
CSV_FILE = 'battery_log.csv'
# ======================
E_max = 92289
Ev_last = E_max
Ei_last = E_max
Et_last = E_max
Percentage = 100
# ======================

def initialize_csv_file():
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["VB", "EU"])

def append_to_csv(VB, EU):
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([VB, EU])

def main():
    global Ev_last, Ei_last, Et_last, Percentage
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
                    VB = data.get("VB")
                    EU = data.get("EU")

                    if VB is not None and EU is not None:
                        append_to_csv(VB, EU)
                        print(f"Logged: {VB}, {EU}")
                    else:
                        print("Missing one or more required keys in JSON.")
                    
                    if(VB > 14.8):
                        Ev = 13009*VB - 115094
                    elif(VB > 14.2):
                        Ev = 96166*VB - 1346076
                    elif(VB > 12.77):
                        Ev = 12606*VB - 159533
                    else:
                        Ev = 314*VB - 2460

                    if Ev > Ev_last:
                        Ev = Ev_last
                    Ev_last = Ev

                    Ei = Et_last-EU
                    if(Percentage < 84 and Percentage > 22):
                        Et = 0.7*Ei + 0.3*Ev
                    else:
                        Et = 0.3*Ei + 0.7*Ev

                    Et_last = Et
                    
                    Percentage = Et/E_max
                    print("Energy = ",Et,"\nPercentage = ", int(Percentage*100), "%")

            except json.JSONDecodeError:
                print("Invalid JSON received.")
            except Exception as e:
                print(f"Error: {e}")

if __name__ == "__main__":
    main()

