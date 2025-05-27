// static/control.js

const client = mqtt.connect(`ws://${location.hostname}:9001`);

client.on('connect', ()=>{
  document.querySelectorAll('button, .arrow').forEach(el=>el.disabled=false);
});
client.on('error', err=>console.error(err));

function publish(topic, payload){
  if(!client.connected) return alert("MQTT not connected");
  client.publish(topic, payload);
}

// Arrow pad
const cmdMap = {
  up:    ['robot/serial','FORWARD'],
  down:  ['robot/serial','BACKWARD'],
  left:  ['robot/serial','LEFT'],
  right: ['robot/serial','RIGHT'],
  stop:  ['robot/serial','STOP']
};
Object.keys(cmdMap).forEach(key=>{
  const el = document.getElementById(key);
  let iid;
  el.onpointerdown = ()=>{ publish(...cmdMap[key]); iid=setInterval(()=>publish(...cmdMap[key]),300); };
  el.onpointerup = el.onpointerleave = ()=>{ clearInterval(iid); };
});

// Button publish (Test + Mode + Auto)
document.querySelectorAll('button[data-mqtt]').forEach(btn=>{
  btn.onclick = ()=>{
    publish(btn.dataset.mqtt, btn.dataset.payload);
  };
});

// PID forms
document.getElementById('innerForm').onsubmit = e=>{
  e.preventDefault();
  const f = e.target;
  const msg = `inner:${f.p.value},${f.d.value},${f.i.value},${f.sp.value}`;
  publish('robot/pid', msg);
};
document.getElementById('outerForm').onsubmit = e=>{
  e.preventDefault();
  const f = e.target;
  const msg = `outer:${f.p.value},${f.d.value},${f.i.value},${f.sp.value},${f.rot.value}`;
  publish('robot/pid', msg);
};

// Key assign
document.getElementById('keyForm').onsubmit = e=>{
  e.preventDefault();
  const loc = e.target.loc.value;
  publish('autonomous', `KEY:${loc}`);
};
