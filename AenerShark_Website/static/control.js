// control.js

// 0. Wait for DOM
window.addEventListener('DOMContentLoaded', () => {

  // 1. Connect to MQTT over WebSockets
  const client = mqtt.connect(`ws://${location.hostname}:9001`);

  client.on('connect', () => {
    console.log('MQTT connected');
    // enable *all* buttons & arrows now that we're connected
    document.querySelectorAll('button, .arrow').forEach(el => el.disabled = false);
  });
  client.on('error', err => console.error('MQTT error:', err));

  // 2. Helper: publish safely & log
  function publish(topic, payload) {
    if (!client.connected) {
      console.warn('MQTT not connected yet; dropping', topic, payload);
      return;
    }
    console.log('→ publish', topic, payload);
    client.publish(topic, payload);
  }

  // 3. Arrow‐pad commands map
  const cmdMap = {
    up:    ['robot/serial', 'FORWARD'],
    down:  ['robot/serial', 'BACKWARD'],
    left:  ['robot/serial', 'LEFT'],
    right: ['robot/serial', 'RIGHT'],
    stop:  ['robot/serial', 'STOP']
  };

  // 4. Pointer events for each arrow
  Object.keys(cmdMap).forEach(key => {
    const el = document.getElementById(key);
    let repeatId;

    el.addEventListener('pointerdown', e => {
      e.preventDefault();
      el.classList.add('active');
      publish(...cmdMap[key]);
      repeatId = setInterval(() => publish(...cmdMap[key]), 300);
    });

    el.addEventListener('pointerup', () => {
      el.classList.remove('active');
      clearInterval(repeatId);
    });
    el.addEventListener('pointerleave', () => {
      el.classList.remove('active');
      clearInterval(repeatId);
    });
  });

  // 5. All .button[data-mqtt] clicks
  document.querySelectorAll('button[data-mqtt]').forEach(btn => {
    btn.addEventListener('click', e => {
      e.preventDefault();
      const topic = btn.dataset.mqtt;
      const payload = btn.dataset.payload;
      publish(topic, payload);
    });
  });

  // 6. PID form submissions (inner + outer)
  document.getElementById('innerForm').addEventListener('submit', e => {
    e.preventDefault();
    const f = e.target;
    const msg = `inner:${f.p.value},${f.d.value},${f.i.value},${f.sp.value}`;
    publish('robot/pid', msg);
  });

  document.getElementById('outerForm').addEventListener('submit', e => {
    e.preventDefault();
    const f = e.target;
    const msg = `outer:${f.p.value},${f.d.value},${f.i.value},${f.sp.value},${f.rot.value}`;
    publish('robot/pid', msg);
  });

  // 7. Key‐location form
  document.getElementById('keyForm').addEventListener('submit', e => {
    e.preventDefault();
    const loc = e.target.loc.value;
    publish('autonomous', `KEY:${loc}`);
  });

  // 8. Keyboard controls (arrow keys + space for STOP)
  let lastKey = null;
  let keyRepeatId = null;
  const keyMap = {
    ArrowUp:    'up',
    ArrowDown:  'down',
    ArrowLeft:  'left',
    ArrowRight: 'right',
    ' ':        'stop'
  };

  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (!action || e.key === lastKey) return;
    lastKey = e.key;
    const el = document.getElementById(action);
    el.classList.add('active');
    publish(...cmdMap[action]);
    if (action !== 'stop') {
      keyRepeatId = setInterval(() => publish(...cmdMap[action]), 300);
    }
  });

  document.addEventListener('keyup', e => {
    if (!lastKey) return;
    const action = keyMap[lastKey];
    const el = document.getElementById(action);
    if (el) el.classList.remove('active');
    clearInterval(keyRepeatId);
    lastKey = null;
  });

}); // end DOMContentLoaded
