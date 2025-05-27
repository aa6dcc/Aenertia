// static/control.js

const client = mqtt.connect('ws://' + location.hostname + ':9001');

client.on('connect', () => {
  document.querySelectorAll('button, .arrow').forEach(el => el.disabled = false);
});
client.on('error', err => console.error('MQTT error:', err));

function publishCommand(topic, msg) {
  if (!client.connected) return alert("MQTT not connected");
  client.publish(topic, msg);
}

const cmdMap = {
  up:    ['robot/serial','FORWARD'],
  down:  ['robot/serial','BACKWARD'],
  left:  ['robot/serial','LEFT'],
  right: ['robot/serial','RIGHT'],
  stop:  ['robot/serial','STOP'],
};

function sendMove(key) {
  let [topic,msg] = cmdMap[key];
  publishCommand(topic, msg);
}

function holdMove(key) {
  sendMove(key);
  return setInterval(()=> sendMove(key), 300);
}

function setActive(el) {
  if (!el.classList.contains('active')) {
    document.querySelectorAll('.arrow.active').forEach(x=>x.classList.remove('active'));
    el.classList.add('active');
  }
}
function clearActive() {
  document.querySelectorAll('.arrow.active').forEach(x=>x.classList.remove('active'));
}

window.addEventListener('DOMContentLoaded', () => {
  // Arrow pad pointer events
  Object.keys(cmdMap).forEach(key => {
    const btn = document.getElementById(key);
    let iid;
    btn.onpointerdown = () => {
      setActive(btn);
      iid = holdMove(key);
    };
    btn.onpointerup = btn.onpointerleave = () => {
      clearActive();
      clearInterval(iid);
    };
  });

  // Keyboard support
  let lastKey, keyIid;
  const keyMap = {
    ArrowUp: 'up',
    ArrowDown: 'down',
    ArrowLeft: 'left',
    ArrowRight:'right',
    ' ': 'stop'
  };
  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (action && e.key !== lastKey) {
      lastKey = e.key;
      const btn = document.getElementById(action);
      setActive(btn);
      sendMove(action);
      if (action !== 'stop') keyIid = setInterval(()=> sendMove(action), 300);
    }
  });
  document.addEventListener('keyup', () => {
    clearActive();
    clearInterval(keyIid);
    lastKey = null;
  });
});
