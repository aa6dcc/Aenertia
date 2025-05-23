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
        <link rel="stylesheet" href="/static/style.css?v=5">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
      </head>
      <body>
        <!-- Floating info bar -->
        <div class="info-bar">
          <div id="time">--:--</div>
          <div id="battery">Battery: --%</div>
        </div>

        <!-- Main control card -->
        <div class="card">
          <img src="/static/images/logo.png" alt="Logo" class="logo">
          <h1>Arrow Pad Control</h1>
          <div class="controller">
            <div class="arrow up"    onclick="press('1')">↑</div>
            <div class="arrow down"  onclick="press('2')">↓</div>
            <div class="arrow left"  onclick="press('3')">←</div>
            <div class="arrow right" onclick="press('4')">→</div>
            <div class="arrow stop"  onclick="press('0')">■</div>
          </div>
        </div>

        <!-- Script -->
        <script>
          let intervalId = null;

          function sendCommand(id) {
            fetch(`/led/${id}`);
          }

          function press(id) {
            clearInterval(intervalId);
            sendCommand(id);
            if (id !== '0') {
              intervalId = setInterval(() => sendCommand(id), 300);
            }
          }

          document.addEventListener("mouseup", () => clearInterval(intervalId));
          document.addEventListener("mouseleave", () => clearInterval(intervalId));

          // Keyboard support
          document.addEventListener("keydown", (e) => {
            const keyMap = {
              ArrowUp: '1',
              ArrowDown: '2',
              ArrowLeft: '3',
              ArrowRight: '4',
              ' ': '0'
            };
            if (keyMap[e.key]) {
              press(keyMap[e.key]);
            }
          });

          // Time and Battery display
          function updateTime() {
            const now = new Date();
            const hours = String(now.getHours()).padStart(2, '0');
            const minutes = String(now.getMinutes()).padStart(2, '0');
            document.getElementById("time").innerText = `${hours}:${minutes}`;
          }

          function updateBattery() {
            if (navigator.getBattery) {
              navigator.getBattery().then(battery => {
                const percent = Math.round(battery.level * 100);
                document.getElementById("battery").innerText = `Battery: ${percent}%`;
              });
            }
          }

          updateTime();
          updateBattery();
          setInterval(updateTime, 10000);
          setInterval(updateBattery, 60000);
        </script>
      </body>
    </html>
    """

