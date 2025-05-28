// control.js

window.addEventListener('DOMContentLoaded', () => {
  console.log('🔥 control.js loaded');

  // 1. MQTT connect
  const client = mqtt.connect(`ws://${location.hostname}:9001`);
  client.on('connect', () => {
    console.log('MQTT connected');
  });
  client.on('error', err => console.error('MQTT error:', err));

  // 2. Safe publish helper
  function publish(topic, payload) {
    if (!client.connected) {
      console.warn('MQTT not connected; dropping', topic, payload);
      return;
    }
    console.log('→ publish', topic, payload);
    client.publish(topic, payload);
  }

  // 3. Arrow-pad (unchanged)
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
      tid = setInterval(() => publish(...args), 300);
    });
    ['pointerup','pointerleave'].forEach(evt => {
      el.addEventListener(evt, () => {
        el.classList.remove('active');
        clearInterval(tid);
      });
    });
  });


  // 4. Mode / Flash / CV / Auto buttons by ID
  const btnConfig = [
    { id: 'btn-manual',     topic: 'mode/set',       payload: 'MANUAL'      },
    { id: 'btn-autonomous', topic: 'mode/set',       payload: 'AUTONOMOUS'  },
    { id: 'btn-test',       topic: 'mode/set',       payload: 'TEST'        },
    { id: 'btn-flash-led',  topic: 'led/control',    payload: 'FLASH'       },
    { id: 'btn-enable-cv',  topic: 'cv/control',     payload: 'ENABLE'      },
    { id: 'btn-disable-cv', topic: 'cv/control',     payload: 'DISABLE'     },
    { id: 'btn-follow',     topic: 'autonomous',     payload: 'FOLLOW'      },
    { id: 'btn-return-key', topic: 'autonomous',     payload: 'RETURN_HOME' },
    { id: 'btn-show-keys',  topic: 'autonomous',     payload: 'SHOW_KEYS'   }
  ];
  btnConfig.forEach(({id, topic, payload}) => {
    const b = document.getElementById(id);
    if (!b) return;
    b.addEventListener('click', e => {
      e.preventDefault();
      publish(topic, payload);
    });
  });

  // Assign Key Location button (needs to read the input)
  const assignBtn = document.getElementById('btn-assign-key');
  if (assignBtn) {
    assignBtn.addEventListener('click', e => {
      e.preventDefault();
      const loc = document.getElementById('key-loc')?.value || '';
      publish('robot/auto/key/assign', loc);
    });
  }

  // 5. PID forms: use the correct input names (pg, dg, ig, sp, rot)
  const inner = document.getElementById('form-inner');
  if (inner) {
    inner.addEventListener('submit', e => {
      e.preventDefault();
      const f = e.target.elements;
      const msg = `inner:${f.pg.value},${f.dg.value},${f.ig.value},${f.sp.value}`;
      publish('robot/pid', msg);
    });
  }

  const outer = document.getElementById('form-outer');
  if (outer) {
    outer.addEventListener('submit', e => {
      e.preventDefault();
      const f = e.target.elements;
      const msg = `outer:${f.pg.value},${f.dg.value},${f.ig.value},${f.sp.value},${f.rot.value}`;
      publish('robot/pid', msg);
    });
  }

  // 6. Keyboard arrows + space STOP
  const keyMap = {
    ArrowUp:   'up',
    ArrowDown: 'down',
    ArrowLeft: 'left',
    ArrowRight:'right',
    ' ':       'stop'
  };
  let lastKey = null, rep = null;
  document.addEventListener('keydown', e => {
    const action = keyMap[e.key];
    if (!action || e.key === lastKey) return;
    lastKey = e.key;
    document.getElementById(action)?.classList.add('active');
    const args = cmdMap[action];
    if (args) {
      publish(...args);
      if (action !== 'stop') rep = setInterval(() => publish(...args), 300);
    }
  });
  document.addEventListener('keyup', e => {
    if (!lastKey) return;
    const action = keyMap[lastKey];
    document.getElementById(action)?.classList.remove('active');
    clearInterval(rep);
    lastKey = null;
  });

}); // end DOMContentLoaded
