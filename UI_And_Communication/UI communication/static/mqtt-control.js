// mqtt-control.js
const brokerUrl = 'ws://172.20.10.9:9001';  // your Pi (update if needed)
const opts = { keepalive: 30, reconnectPeriod: 1000 };
const client = mqtt.connect(brokerUrl, opts);

// ---- MQTT Event Handlers ----
client.on('connect', () => {
  console.log('MQTT connected');
  document.getElementById('mqtt-status').innerText = 'MQTT: Connected';
  client.subscribe('robot/battery');
  client.subscribe('robot/keys');
});

client.on('message', (topic, message) => {
  const msg = message.toString();
  if (topic === 'robot/battery') {
    document.getElementById('battery').innerText = `Battery: ${msg}%`;
  } else if (topic === 'robot/keys') {
    const keys = JSON.parse(msg);
    const keyList = document.getElementById('key-list');
    keyList.innerHTML = '<ul>' + keys.map(k =>
      `<li><button class="key-button" onclick="assignKeyLocation('${k}')">${k}</button></li>`
    ).join('') + '</ul>';
  }
});

client.on('error', err => {
  console.error('MQTT error', err);
  document.getElementById('mqtt-status').innerText = 'MQTT: Error';
  document.getElementById('battery').innerText = 'Battery: --%';
});

client.on('close', () => {
  console.warn('MQTT connection closed');
  document.getElementById('mqtt-status').innerText = 'MQTT: Disconnected';
  document.getElementById('battery').innerText = 'Battery: --%';
});

// ---- Helper to publish ----
function pub(topic, payload) {
  client.publish(topic, payload);
}

// ---- Click on location to assign it ----
function assignKeyLocation(loc) {
  pub('robot/auto/key/assign', loc);
}

// ---- Clock update ----
setInterval(() => {
  const now = new Date();
  document.getElementById('time').innerText = now.toLocaleTimeString([], {hour: '2-digit', minute: '2-digit'});
}, 10000);

// ---- Arrow Control Commands ----
const commandMap = {
  up: 'up', down: 'down', left: 'left', right: 'right', stop: 'stop',
  'up-left': 'up-left', 'up-right': 'up-right', 'down-left': 'down-left', 'down-right': 'down-right'
};
const arrowTopic = 'robot/manual/command';

// ---- Mouse-based arrow controls ----
['up','down','left','right'].forEach(dir => {
  const btn = document.getElementById(dir);
  if (!btn) return;
  btn.onmousedown = () => {
    activeKeys.add(dir);
    updateMovement();
  };
  btn.onmouseup = btn.onmouseleave = () => {
    activeKeys.delete(dir);
    updateMovement();
  };
});

// ---- Keyboard-based arrow controls with diagonal support ----
const keyMap = {
  ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right'
};
const activeKeys = new Set();
let movementInterval = null;

document.addEventListener('keydown', e => {
  const dir = keyMap[e.key];
  if (!dir) return;
  if (!activeKeys.has(dir)) {
    activeKeys.add(dir);
    updateMovement();
  }
});

document.addEventListener('keyup', e => {
  const dir = keyMap[e.key];
  if (!dir) return;
  activeKeys.delete(dir);
  updateMovement();
});

function updateMovement() {
  if (movementInterval) clearInterval(movementInterval);
  if (activeKeys.size === 0) {
    pub(arrowTopic, commandMap.stop);
    document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
    return;
  }

  document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
  activeKeys.forEach(d => {
    const b = document.getElementById(d);
    if (b) b.classList.add('active');
  });

  sendMovement();
  movementInterval = setInterval(sendMovement, 200);
}

function sendMovement() {
  const hasUp = activeKeys.has('up');
  const hasDown = activeKeys.has('down');
  const hasLeft = activeKeys.has('left');
  const hasRight = activeKeys.has('right');

  let cmd;
  if (hasUp && hasLeft) cmd = commandMap['up-left'];
  else if (hasUp && hasRight) cmd = commandMap['up-right'];
  else if (hasDown && hasLeft) cmd = commandMap['down-left'];
  else if (hasDown && hasRight) cmd = commandMap['down-right'];
  else if (hasUp) cmd = commandMap.up;
  else if (hasDown) cmd = commandMap.down;
  else if (hasLeft) cmd = commandMap.left;
  else if (hasRight) cmd = commandMap.right;
  else cmd = commandMap.stop;

  pub(arrowTopic, cmd);
}

// ---- Other button mappings ----
const buttonActions = [
  { id: 'btn-manual', topic: 'robot/mode', payload: 'manual' },
  { id: 'btn-autonomous', topic: 'robot/mode', payload: 'autonomous' },
  { id: 'btn-test', topic: 'robot/mode', payload: 'test' },
  { id: 'btn-flash-led', topic: 'robot/led', payload: 'flash' },
  { id: 'btn-enable-cv', topic: 'robot/cv', payload: 'enable' },
  { id: 'btn-disable-cv', topic: 'robot/cv', payload: 'disable' },
  { id: 'btn-follow', topic: 'robot/auto', payload: 'follow' },
  { id: 'btn-return-key', topic: 'robot/auto', payload: 'return' },
  { id: 'btn-show-keys', topic: 'robot/auto', payload: 'show_keys' }
];
buttonActions.forEach(({id, topic, payload}) => {
  const btn = document.getElementById(id);
  if (!btn) return;
  btn.onclick = () => pub(topic, payload);
});

// ---- Assign key location ----
const assignBtn = document.getElementById('btn-assign-key');
if (assignBtn) assignBtn.onclick = () => {
  const loc = document.getElementById('key-loc').value;
  if (loc.trim()) pub('robot/auto/key/assign', loc.trim());
};

// ---- PID Form Handling ----
const formInner = document.getElementById('form-inner');
if (formInner) formInner.onsubmit = e => {
  e.preventDefault();
  const d = new FormData(e.target);
  pub('robot/pid/inner', JSON.stringify({
    pg: +d.get('pg'),
    dg: +d.get('dg'),
    ig: +d.get('ig'),
    sp: +d.get('sp')
  }));
};

const formOuter = document.getElementById('form-outer');
if (formOuter) formOuter.onsubmit = e => {
  e.preventDefault();
  const d = new FormData(e.target);
  pub('robot/pid/outer', JSON.stringify({
    pg: +d.get('pg'),
    dg: +d.get('dg'),
    ig: +d.get('ig'),
    sp: +d.get('sp'),
    rot: +d.get('rot')
  }));
};

// === VOICE RECOGNITION ===
const recognition = new window.webkitSpeechRecognition();
recognition.lang = 'en-US';
recognition.continuous = false;

const voiceBtn = document.getElementById('start-voice');
voiceBtn.onmousedown = () => {
  console.log('Voice button pressed → start recognition');
  recognition.start();
};
voiceBtn.onmouseup = () => {
  console.log('Voice button released → stop recognition');
  recognition.stop();
};

recognition.onresult = function(event) {
  const transcript = event.results[0][0].transcript;
  console.log("Voice result received:", transcript);
  sendToChatGPT(transcript);
};

function sendToChatGPT(commandText) {
  console.log("Sending to Flask server:", commandText);
  fetch('/interpret', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ command: commandText })
  })
  .then(res => res.json())
  .then(data => {
    const cmd = data.result;
    console.log("GPT Interpreted:", cmd);

    // Send via MQTT just like buttons
    if (cmd === "follow" || cmd === "return" || cmd === "stop") {
      console.log(`Publishing ${cmd} to robot/auto`);
      client.publish("robot/auto", cmd);
    } else if (cmd === "manual" || cmd === "autonomous") {
      console.log(`Publishing ${cmd} to robot/mode`);
      client.publish("robot/mode", cmd);
    } else {
      alert("Sorry, I couldn't understand that command.");
      console.warn("GPT returned unrecognized command:", cmd);
    }
  })
  .catch(err => {
    console.error("[DEBUG] Error contacting GPT API:", err);
  });
}
