from fastapi import FastAPI, Form
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.publish as publish

app = FastAPI()

# Mount static files (CSS, JS, images)
app.mount("/static", StaticFiles(directory="static"), name="static")

# Store submitted values
inner_history = []
outer_history = []

# MQTT LED control (arrow pad + flash)
@app.get("/led/{id}")
def control_led(id: int):
    topic = "led/control"
    msg = {
        1: "UP", 2: "DOWN", 3: "LEFT", 4: "RIGHT"
    }.get(id, "OFF")
    publish.single(topic, msg, hostname="localhost")
    return {"status": f"Command sent: {msg}"}

@app.get("/led/flash")
def flash_led():
    publish.single("led/control", "FLASH", hostname="localhost")
    return {"status": "Flashing LED"}

@app.get("/dashboard", response_class=HTMLResponse)
def dashboard():
    return render_dashboard()

@app.post("/submit_inner", response_class=HTMLResponse)
def submit_inner(pg: str = Form(...), dg: str = Form(...), ig: str = Form(...), sp: str = Form(...)):
    inner_history.append([pg, dg, ig, sp])
    return render_dashboard()

@app.post("/submit_outer", response_class=HTMLResponse)
def submit_outer(pg: str = Form(...), dg: str = Form(...), ig: str = Form(...), sp: str = Form(...), rot: str = Form(...)):
    outer_history.append([pg, dg, ig, sp, rot])
    return render_dashboard()

def render_dashboard():
    def table(rows, headers):
        html = "<table><tr>" + "".join(f"<th>{h}</th>" for h in headers) + "</tr>"
        for row in rows:
            html += "<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>"
        html += "</table>"
        return html

    inner_table = table(inner_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint"])
    outer_table = table(outer_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint", "Rotation Setpoint"])

    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Dashboard</title>
        <link rel='stylesheet' href='/static/style.css'>
        <script src="/static/control.js" defer></script>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
    </head>
    <body>
        <div class="info-bar">
            <div id="time">--:--</div>
            <div id="battery">Battery: --%</div>
        </div>

        <div class="tabs">
            <button onclick="showTab('pad')">Arrow Pad</button>
            <button onclick="showTab('flash')">Flash LED</button>
            <button onclick="showTab('pid')">Control Values</button>
        </div>

        <div id="pad" class="tab active">
            <div class="card">
                <img src="/static/images/logo.png" class="logo">
                <h1>Arrow Pad Control</h1>
                <div class="controller">
                    <div class="arrow up" id="up">↑</div>
                    <div class="arrow down" id="down">↓</div>
                    <div class="arrow left" id="left">←</div>
                    <div class="arrow right" id="right">→</div>
                    <div class="arrow stop" id="stop">■</div>
                </div>
            </div>
        </div>

        <div id="flash" class="tab">
            <div class="card">
                <h1>Flash LED</h1>
                <form action="/led/flash" method="get">
                    <button type="submit" class="button">Flash LED</button>
                </form>
            </div>
        </div>

        <div id="pid" class="tab">
            <div class="card">
                <h1>Control Values</h1>
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
        </div>

        <script>
        function showTab(id) {{
            document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
            document.getElementById(id).classList.add('active');
        }}

        function updateTime() {{
            const now = new Date();
            const hours = String(now.getHours()).padStart(2, '0');
            const minutes = String(now.getMinutes()).padStart(2, '0');
            document.getElementById("time").innerText = `${{hours}}:${{minutes}}`;
        }}

        function updateBattery() {{
            if (navigator.getBattery) {{
                navigator.getBattery().then(battery => {{
                    const percent = Math.round(battery.level * 100);
                    document.getElementById("battery").innerText = `Battery: ${{percent}}%`;
                }});
            }}
        }}

        updateTime();
        updateBattery();
        setInterval(updateTime, 10000);
        setInterval(updateBattery, 60000);
        </script>
    </body>
    </html>
    """
