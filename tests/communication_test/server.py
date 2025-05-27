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
        <link rel="stylesheet" href="/static/style.css?v=6">
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

          function highlightArrow(className) {
            const btn = document.querySelector(`.arrow.${className}`);
            if (!btn) return;
            btn.classList.add("active");
            setTimeout(() => btn.classList.remove("active"), 200);
          }

          document.addEventListener("mouseup", () => clearInterval(intervalId));
          document.addEventListener("mouseleave", () => clearInterval(intervalId));

          // Keyboard support + animation
          document.addEventListener("keydown", (e) => {
            const keyMap = {
              ArrowUp: ['1', 'up'],
              ArrowDown: ['2', 'down'],
              ArrowLeft: ['3', 'left'],
              ArrowRight: ['4', 'right'],
              ' ': ['0', 'stop']
            };
            const action = keyMap[e.key];
            if (action) {
              press(action[0]);
              highlightArrow(action[1]);
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
inner_history = []
outer_history = []

@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard():
    return render_dashboard()

@app.post("/submit_inner", response_class=HTMLResponse)
async def submit_inner(
    pg: str = Form(...),
    dg: str = Form(...),
    ig: str = Form(...),
    sp: str = Form(...)
):
    inner_history.append([pg, dg, ig, sp])
    return render_dashboard()

@app.post("/submit_outer", response_class=HTMLResponse)
async def submit_outer(
    pg: str = Form(...),
    dg: str = Form(...),
    ig: str = Form(...),
    sp: str = Form(...),
    rot: str = Form(...)
):
    outer_history.append([pg, dg, ig, sp, rot])
    return render_dashboard()

def render_dashboard():
    def table(rows, headers):
        html = "<table border='1'><tr>" + "".join(f"<th>{h}</th>" for h in headers) + "</tr>"
        for row in rows:
            html += "<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>"
        html += "</table>"
        return html

    inner_table = table(inner_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint"])
    outer_table = table(outer_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint", "Rotation Setpoint"])

    return f"""
    <html>
    <head>
    <style>
        .tab {{ display: none; }}
        .tab.active {{ display: block; }}
        button.tablink {{
            margin: 5px;
            padding: 10px 20px;
        }}
    </style>
    <script>
        function showTab(id) {{
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.getElementById(id).classList.add('active');
        }}
    </script>
    </head>
    <body>
        <h1>Robot Control Dashboard</h1>
        <button class="tablink" onclick="showTab('flash')">Flash LED</button>
        <button class="tablink" onclick="showTab('control')">Control Values</button>

        <div id="flash" class="tab active">
            <h2>Flash LED</h2>
            <form action="/led/flash" method="get">
                <button type="submit">Flash LED</button>
            </form>
        </div>

        <div id="control" class="tab">
            <h2>Inner Loop</h2>
            <form action="/submit_inner" method="post">
                Proportional Gain <input name="pg"><br>
                Derivative Gain <input name="dg"><br>
                Integral Gain <input name="ig"><br>
                Setpoint <input name="sp"><br>
                <input type="submit" value="Submit">
            </form>
            <h3>Received PID Tuning Values</h3>
            {inner_table}
            <h2>Outer Loop</h2>
            <form action="/submit_outer" method="post">
                Proportional Gain <input name="pg"><br>
                Derivative Gain <input name="dg"><br>
                Integral Gain <input name="ig"><br>
                Setpoint <input name="sp"><br>
                Rotation Setpoint <input name="rot"><br>
                <input type="submit" value="Submit">
            </form>
            <h3>Received PID Tuning Values</h3>
            {outer_table}
        </div>
    </body>
    </html>
    """

