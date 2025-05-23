from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.publish as publish

app = FastAPI()

# Mount static files (JS, CSS, images)
app.mount("/static", StaticFiles(directory="static"), name="static")

# MQTT-based LED control
@app.get("/led/{id}")
def control_led(id: int):
    if id == 1:
        publish.single("led/control", "UP", hostname="localhost")
    elif id == 2:
        publish.single("led/control", "DOWN", hostname="localhost")
    elif id == 3:
        publish.single("led/control", "LEFT", hostname="localhost")
    elif id == 4:
        publish.single("led/control", "RIGHT", hostname="localhost")
    else:
        publish.single("led/control", "OFF", hostname="localhost")
    return {"status": f"Command sent: LED{id}"}

# Simple button-based control (original)
@app.get("/", response_class=HTMLResponse)
def index():
    return """
    <h1>LED Control</h1>
    <a href="/led/1"><button>LED 1</button></a>
    <a href="/led/2"><button>LED 2</button></a>
    <a href="/led/0"><button>OFF</button></a>
    """

# New arrow pad interface
@app.get("/pad", response_class=HTMLResponse)
def arrow_pad():
    return """
    <!DOCTYPE html>
    <html>
      <head>
        <title>Arrow Pad Control</title>
        <link rel="stylesheet" type="text/css" href="/static/style.css?v=2">
      </head>
      <body>
        <img src="/static/images/logo.png" alt="Logo" class="logo">
        <h1>Arrow Pad Control</h1>
        <div class="controller">
          <img id="up"    src="/static/images/up.png" class="up">
          <img id="down"  src="/static/images/down.png" class="down">
          <img id="left"  src="/static/images/left.png" class="left">
          <img id="right" src="/static/images/right.png" class="right">
          <img id="stop"  src="/static/images/stop.png" class="stop">
        </div>
        <script src="/static/control.js"></script>
      </body>
    </html>
    """

