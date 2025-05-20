import os

# Capture image using libcamera
exit_code = os.system("libcamera-still -n -o snapshot.jpg")

# Print OK or ERROR for Node.js to read
if exit_code == 0:
    print("OK")
else:
    print("ERROR")
