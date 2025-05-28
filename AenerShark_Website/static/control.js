// control.js

window.addEventListener('DOMContentLoaded', () => {
  console.log('🔥 control.js loaded');

  // 1. MQTT connect
  const client = mqtt.connect(`ws://${location.hostname}:9001`);
  client.on('connect', () => {
    console.log('MQTT connected');
  });
  client.on('error', err => console.error('MQTT error:', err));

  // 2. Publish helper
  function publish(topic, payload) {
    if (!client.connected) {
      console.warn('MQTT not connected; dropping', topic, payload);
      return;
    }
    console.log('→ publish', topic, payload);
    client.publish(topic, payload);
  }

  // 3. Arrow-pad map + events (unchanged)
  const cmdMap = {
    up:    ['robot/serial','FORWARD'],
    down:  ['robot/serial','BACKWARD'],
    left:  ['robot/serial','LEFT'],
    right: ['robot/serial','RIGHT'],
    stop:  ['robot/serial','STOP']
  };
  Object.entries(cmdMap).forEach(([key, args]) => {
    const el = document.getElementById(key);
    if (!el) return;
    let tid;
    el.addEventListener('pointerdown', e => {
      e.preventDefault();
      el.classList.add('active');
      publish(...args);
      tid = setInterval(()=>publish(...args), 300);
    });
    ['pointerup','pointerleave'].forEach(evt =>
      el.addEventListener(evt, ()=>{
        el.classList.remove('active');
        clearInterval(tid);
      })
    );
  });

  // 4. Wire up ALL data-mqtt buttons
  document.querySelectorAll('button[data-mqtt]').forEach(btn => {
    btn.addEventListener('click', e => {
      e.preventDefault();
      const topic   = btn.getAttribute('data-mqtt');
      const payload = btn.getAttribute('data-payload');
      publish(topic, payload);
    });
  });

  // 5. PID forms — use form.elements[...] to get values
  const innerForm = document.getElementById('innerForm')
                 || document.getElementById('form-inner');
  if (innerForm) {
    innerForm.addEventListener('submit', e => {
      e.preventDefault();
      const f = e.target.elements;
      const msg = `inner:${f.p.value},${f.d.value},${f.i.value},${f.sp.value}`;
      publish('robot/pid', msg);
    });
  }

  const outerForm = document.getElementById('outerForm')
                 || document.getElementById('form-outer');
  if (outerForm) {
    outerForm.addEventListener('submit', e => {
      e.preventDefault();
      const f = e.target.elements;
      const msg = `outer:${f.p.value},${f.d.value},${f.i.value},${f.sp.value},${f.rot.value}`;
      publish('robot/pid', msg);
    });
  }

  // 6. Key-location form
  const keyForm = document.getElementById('keyForm')
                || document.getElementById('form-key');
  if (keyForm) {
    keyForm.addEventListener('submit', e => {
      e.preventDefault();
      const loc = e.target.elements.loc.value;
      publish('autonomous', `KEY:${loc}`);
    });
  }

  // 7. Keyboard arrows + space
  const keyMap = { ArrowUp:'up', ArrowDown:'down', ArrowLeft:'left', ArrowRight:'right', ' ':'stop' };
  let lastKey = null, rep = null;
  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (!action || e.key === lastKey) return;
    lastKey = e.key;
    document.getElementById(action)?.classList.add('active');
    publish(...(cmdMap[action]||[]));
    if (action !== 'stop') rep = setInterval(()=>publish(...cmdMap[action]),300);
  });
  document.addEventListener('keyup', e => {
    if (!lastKey) return;
    const action = keyMap[lastKey];
    document.getElementById(action)?.classList.remove('active');
    clearInterval(rep);
    lastKey = null;
  });

}); // end DOMContentLoaded
