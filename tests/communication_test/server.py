from fastapi import FastAPI
import paho.mqtt.publish as publish

app = FastAPI()

@app.get("/led/{id}")
def control_led(id: int):
    if id == 1:
        publish.single("led/control", "LED1", hostname="localhost")
    elif id == 2:
        publish.single("led/control", "LED2", hostname="localhost")
    else:
        publish.single("led/control", "OFF", hostname="localhost")
    return {"status": f"Command sent: LED{id}"}


# Start it using uvicorn led_api:app --host 0.0.0.0 --port 8000
