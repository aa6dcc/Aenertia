// static/control.js

// 1. MQTT over WebSocket
const client = mqtt.connect('ws://' + location.hostname + ':9001');

client.on('connect', () => {
  console.log('🔌 MQTT connected');
  // enable all buttons once connected
  document.querySelectorAll('button, .arrow').forEach(el => el.disabled = false);
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
  publishCommand('robot/led', 'flash');
}

// 5. Movement commands
const commandMap = {
  up:      ['FORWARD',  'robot/serial'],
  down:    ['BACKWARD', 'robot/serial'],
  left:    ['LEFT',     'robot/serial'],
  right:   ['RIGHT',    'robot/serial'],
  stop:    ['STOP',     'robot/serial'],
};

function sendMove(cmd) {
  publishCommand(commandMap[cmd][1], commandMap[cmd][0]);
}

function holdMove(cmd) {
  sendMove(cmd);
  return setInterval(() => sendMove(cmd), 300);
}

// 6. Active‐state helpers
function setActive(el) {
  if (!el.classList.contains('active')) {
    clearActive();
    el.classList.add('active');
  }
}
function clearActive() {
  document.querySelectorAll('.arrow.active').forEach(el => el.classList.remove('active'));
}

// 7. Wire up pointer (mouse/touch) events
window.addEventListener('DOMContentLoaded', () => {
  // PID forms
  document.getElementById('innerForm').onsubmit = e => { e.preventDefault(); submitPID('inner'); };
  document.getElementById('outerForm').onsubmit = e => { e.preventDefault(); submitPID('outer'); };

  // LED button
  document.getElementById('flashLedBtn').onclick = flashLED;

  // D-pad pointer & hold
  Object.keys(commandMap).forEach(key => {
    const btn = document.getElementById(key);
    let intervalId = null;

    btn.onpointerdown = () => {
      setActive(btn);
      intervalId = holdMove(key);
    };
    btn.onpointerup = btn.onpointerleave = () => {
      clearActive();
      clearInterval(intervalId);
    };
  });

  // Keyboard control
  let keyInterval = null;
  let lastKey = null;
  const keyMap = {
    ArrowUp:    'up',
    ArrowDown:  'down',
    ArrowLeft:  'left',
    ArrowRight: 'right',
    ' ':        'stop'
  };

  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (action && e.key !== lastKey) {
      lastKey = e.key;
      const btn = document.getElementById(action);
      setActive(btn);
      sendMove(action);
      if (action !== 'stop') {
        keyInterval = setInterval(() => sendMove(action), 300);
      }
    }
  });

  document.addEventListener('keyup', () => {
    clearActive();
    clearInterval(keyInterval);
    lastKey = null;
  });
});
