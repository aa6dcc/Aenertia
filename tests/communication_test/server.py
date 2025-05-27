from fastapi import FastAPI, Form, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import paho.mqtt.publish as publish

app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")

inner_history = []
outer_history = []
key_locations = []

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

@app.get("/cv/enable")
def enable_cv():
    publish.single("cv/control", "ENABLE", hostname="localhost")
    return {"status": "CV Enabled"}

@app.get("/cv/disable")
def disable_cv():
    publish.single("cv/control", "DISABLE", hostname="localhost")
    return {"status": "CV Disabled"}

@app.get("/autonomous/follow")
def autonomous_follow():
    publish.single("autonomous", "FOLLOW", hostname="localhost")
    return {"status": "Follow Me activated"}

@app.get("/autonomous/return")
def autonomous_return():
    publish.single("autonomous", "RETURN_HOME", hostname="localhost")
    return {"status": "Returning Home"}

@app.post("/autonomous/key_location", response_class=HTMLResponse)
def assign_location(loc: str = Form(...)):
    key_locations.append(loc)
    publish.single("autonomous", f"KEY:{loc}", hostname="localhost")
    return render_dashboard(show_keys=False)

@app.get("/autonomous/set_key/{loc}")
def set_existing_location(loc: str):
    publish.single("autonomous", f"KEY:{loc}", hostname="localhost")
    return render_dashboard(show_keys=False)

@app.get("/dashboard", response_class=HTMLResponse)
def dashboard():
    return render_dashboard(show_keys=False)

@app.get("/dashboard/show_keys", response_class=HTMLResponse)
def dashboard_show_keys():
    return render_dashboard(show_keys=True)

@app.get("/set_mode/{mode}")
def set_mode(mode: str):
    publish.single("mode/set", mode.upper(), hostname="localhost")
    return {"status": f"Mode set to {mode}"}

@app.post("/submit_inner", response_class=HTMLResponse)
def submit_inner(pg: str = Form(...), dg: str = Form(...), ig: str = Form(...), sp: str = Form(...)):
    inner_history.append([pg, dg, ig, sp])
    return render_dashboard(show_keys=False)

@app.post("/submit_outer", response_class=HTMLResponse)
def submit_outer(pg: str = Form(...), dg: str = Form(...), ig: str = Form(...), sp: str = Form(...), rot: str = Form(...)):
    outer_history.append([pg, dg, ig, sp, rot])
    return render_dashboard(show_keys=False)

def render_dashboard(show_keys=False):
    def table(rows, headers):
        html = "<table><tr>" + "".join(f"<th>{h}</th>" for h in headers) + "</tr>"
        for row in rows:
            html += "<tr>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>"
        html += "</table>"
        return html

    inner_table = table(inner_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint"])
    outer_table = table(outer_history, ["Proportional Gain", "Derivative Gain", "Integral Gain", "Setpoint", "Rotation Setpoint"])
    key_table = "<table><tr><th>Saved Key Locations</th></tr>"
    for loc in key_locations:
        key_table += f"<tr><td><a href='/autonomous/set_key/{loc}'><button class='button'>{loc}</button></a></td></tr>"
    key_table += "</table>"

    key_block = f"<h3>Key Locations</h3>{key_table}" if show_keys else ""

    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Robot Dashboard</title>
        <link rel='stylesheet' href='/static/style.css'>
        <script src="/static/control.js" defer></script>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            .mode-bar-top-left {{
                position: absolute;
                top: 20px;
                left: 20px;
                display: flex;
                flex-direction: column;
                gap: 10px;
                z-index: 1000;
            }}
            .mode-bar-top-left form {{ margin: 0; }}
            .auto-grid {{ display: flex; justify-content: space-between; gap: 20px; }}
            .auto-column {{ flex: 1; }}
        </style>
    </head>
    <body>
        <div class="info-bar">
            <div id="time">--:--</div>
            <div id="battery">Battery: --%</div>
        </div>

        <div class="mode-bar-top-left">
            <form action="/set_mode/manual" method="get"><button class="button">Set Manual</button></form>
            <form action="/set_mode/autonomous" method="get"><button class="button">Set Autonomous</button></form>
            <form action="/set_mode/test" method="get"><button class="button">Set Test Mode</button></form>
        </div>

        <div class="tabs">
            <button onclick="showTab('pad')">Manual Control</button>
            <button onclick="showTab('flash')">Test</button>
            <button onclick="showTab('pid')">PID Values</button>
            <button onclick="showTab('auto')">Autonomous</button>
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
                <form action="/cv/enable" method="get">
                    <button type="submit" class="button">Enable CV</button>
                </form>
                <form action="/cv/disable" method="get">
                    <button type="submit" class="button">Disable CV</button>
                </form>
            </div>
        </div>

        <div id="pid" class="tab">
            <div class="card">
                <h1>PID Values</h1>
                <h2>Inner Loop</h2>
                <form action="/submit_inner" method="post">
                    Proportional Gain <input name="pg"><br>
                    Derivative Gain <input name="dg"><br>
                    Integral Gain <input name="ig"><br>
                    Setpoint <input name="sp"><br>
                    <input type="submit" value="Submit" class="button">
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
                    <input type="submit" value="Submit" class="button">
                </form>
                <h3>Received PID Tuning Values</h3>
                {outer_table}
            </div>
        </div>

        <div id="auto" class="tab">
            <div class="card">
                <h1>Autonomous Control</h1>
                <div class="auto-grid">
                    <div class="auto-column">
                        <form action="/autonomous/follow" method="get">
                            <button class="button" type="submit">Follow Me</button>
                        </form>
                    </div>
                    <div class="auto-column">
                        <form action="/autonomous/key_location" method="post">
                            Assign Key Location: <input name="loc">
                            <button class="button" type="submit">Assign</button>
                        </form>
                        <form action="/autonomous/return" method="get">
                            <button class="button" type="submit">Return to Key Location</button>
                        </form>
                        <form action="/dashboard/show_keys" method="get">
                            <button class="button" type="submit">Show Key Locations</button>
                        </form>
                        {key_block}
                    </div>
                </div>
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
