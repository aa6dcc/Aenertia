// static/control.js

// 1. MQTT over WebSocket
const client = mqtt.connect('ws://' + location.hostname + ':9001');

client.on('connect', () => {
  console.log('🔌 MQTT connected');
  // enable all buttons once connected
  document.querySelectorAll('button').forEach(btn => btn.disabled = false);
});
client.on('error', err => console.error('MQTT error:', err));

// 2. Publish utility
function publishCommand(topic, message) {
  if (client.connected) {
    client.publish(topic, message);
  } else {
    alert("MQTT not connected yet.");
  }
}

// 3. PID submission
function submitPID(loopType) {
  if (!client.connected) {
    alert("MQTT not connected yet.");
    return;
  }
  const P   = document.getElementById(loopType + 'P').value || '0';
  const D   = document.getElementById(loopType + 'D').value || '0';
  const I   = document.getElementById(loopType + 'I').value || '0';
  const Set = document.getElementById(loopType + 'Set').value || '0';
  const msg = `${loopType}:${P},${D},${I},${Set}`;
  client.publish('robot/pid', msg);
}

// 4. Flash LED
function flashLED() {
  if (!client.connected) {
    alert("MQTT not connected yet.");
    return;
  }
  client.publish('robot/led', 'flash');
}

// 5. Directional controls
function startMove(direction) {
  publishCommand('robot/serial', direction);
}
function stopMove() {
  publishCommand('robot/serial', 'STOP');
}

// 6. Hook up form “onsubmit” handlers
window.addEventListener('DOMContentLoaded', () => {
  document.getElementById('submitInner').form.onsubmit = e => {
    e.preventDefault();
    submitPID('inner');
  };
  document.getElementById('submitOuter').form.onsubmit = e => {
    e.preventDefault();
    submitPID('outer');
  };
});
