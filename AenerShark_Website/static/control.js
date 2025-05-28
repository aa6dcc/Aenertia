// control.js

// 0. Fire once the DOM is fully parsed
window.addEventListener('DOMContentLoaded', () => {
  console.log('🔥 control.js loaded');

  // 1. Connect to MQTT over WebSockets
  const client = mqtt.connect(`ws://${location.hostname}:9001`);
  client.on('connect', () => {
    console.log('MQTT connected');
    document.querySelectorAll('button, .arrow').forEach(el => el.disabled = false);
  });
  client.on('error', err => console.error('MQTT error:', err));

  // 2. Safe publish helper
  function publish(topic, payload) {
    if (!client.connected) {
      return console.warn('MQTT not connected; dropping', topic, payload);
    }
    console.log('→ publish', topic, payload);
    client.publish(topic, payload);
  }

  // 3. Arrow-pad commands map
  const cmdMap = {
    up:    ['robot/serial', 'FORWARD'],
    down:  ['robot/serial', 'BACKWARD'],
    left:  ['robot/serial', 'LEFT'],
    right: ['robot/serial', 'RIGHT'],
    stop:  ['robot/serial', 'STOP']
  };

  // 4. Pointer events for arrows
  Object.keys(cmdMap).forEach(key => {
    const el = document.getElementById(key);
    if (!el) return;
    let repeatId;

    el.addEventListener('pointerdown', e => {
      e.preventDefault();
      el.classList.add('active');
      publish(...cmdMap[key]);
      repeatId = setInterval(() => publish(...cmdMap[key]), 300);
    });
    ['pointerup','pointerleave'].forEach(evt => {
      el.addEventListener(evt, () => {
        el.classList.remove('active');
        clearInterval(repeatId);
      });
    });
  });

  // 5. All data-mqtt buttons
  document.querySelectorAll('button[data-mqtt]').forEach(btn => {
    btn.addEventListener('click', e => {
      e.preventDefault();
      publish(btn.dataset.mqtt, btn.dataset.payload);
    });
  });

  // Helper to bind form submits without crashing if IDs differ
  function bindForm(ids, handler) {
    for (const id of ids) {
      const form = document.getElementById(id);
      if (form) {
        form.addEventListener('submit', handler);
        return;
      }
    }
  }

  // 6. PID forms
  bindForm(['innerForm','form-inner'], e => {
    e.preventDefault();
    const f = e.target;
    const msg = `inner:${f.p?.value},${f.d?.value},${f.i?.value},${f.sp?.value}`;
    publish('robot/pid', msg);
  });

  bindForm(['outerForm','form-outer'], e => {
    e.preventDefault();
    const f = e.target;
    const msg = `outer:${f.p?.value},${f.d?.value},${f.i?.value},${f.sp?.value},${f.rot?.value}`;
    publish('robot/pid', msg);
  });

  // 7. Key-location form
  bindForm(['keyForm','form-key','form-auto'], e => {
    e.preventDefault();
    const loc = e.target.loc?.value;
    publish('autonomous', `KEY:${loc}`);
  });

  // 8. Keyboard arrows + space as STOP
  const keyMap = { ArrowUp:'up', ArrowDown:'down', ArrowLeft:'left', ArrowRight:'right', ' ':'stop' };
  let lastKey = null, keyRepeat = null;

  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (!action || e.key === lastKey) return;
    lastKey = e.key;
    const el = document.getElementById(action);
    if (el) el.classList.add('active');
    publish(...(cmdMap[action]||[]));
    if (action !== 'stop') {
      keyRepeat = setInterval(() => publish(...cmdMap[action]), 300);
    }
  });

  document.addEventListener('keyup', e => {
    if (!lastKey) return;
    const action = keyMap[lastKey];
    const el = document.getElementById(action);
    if (el) el.classList.remove('active');
    clearInterval(keyRepeat);
    lastKey = null;
  });
});
