from fastapi import FastAPI
import paho.mqtt.publish as publish
from fastapi.responses import HTMLResponse

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


# Define a route for no / (test the UI before running mqtt)
@app.get("/", response_class=HTMLResponse)
def index():
    return """
    <h1>LED Control</h1>
    <a href="/led/1"><button>LED 1</button></a>
    <a href="/led/2"><button>LED 2</button></a>
    <a href="/led/0"><button>OFF</button></a>
    """

# Start it using uvicorn led_api:app --host 0.0.0.0 --port 8000
