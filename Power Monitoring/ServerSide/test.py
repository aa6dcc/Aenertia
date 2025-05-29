import serial
import time

# Serial port configuration — change as needed
PORT = 'COM4'           # Your Arduino/adapter port
BAUD = 9600
TIMEOUT = 2             # seconds to wait for response

# Test message to send
TEST_MESSAGE = "echo_test_123"

def echo_test():
    try:
        with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
            print(f"Sending: {TEST_MESSAGE}")
            ser.write((TEST_MESSAGE + "\n").encode())

            start_time = time.time()
            received = ""

            while time.time() - start_time < TIMEOUT:
                if ser.in_waiting:
                    received += ser.read(ser.in_waiting).decode(errors='ignore')
                    if TEST_MESSAGE in received:
                        print("✅ Echo test passed.")
                        return True
                time.sleep(0.05)

            print("❌ Echo test failed (timeout).")
            return False

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False

if __name__ == "__main__":
    echo_test()
