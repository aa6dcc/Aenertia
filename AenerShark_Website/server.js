const express = require('express');
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const mqtt = require('mqtt');

const app = express();
const PORT = 5000;

let pidValues = {
  inner: [],
  outer: []
};

// Static file serving
app.use(express.static(path.join(__dirname)));

// MQTT setup
const mqttClient = mqtt.connect('mqtt://localhost');

mqttClient.on('connect', () => {
  console.log(' MQTT connected');
  mqttClient.subscribe(['robot/pid', 'robot/led']);
});

mqttClient.on('message', (topic, message) => {
  const msg = message.toString();

  if (topic === 'robot/pid') {
    const [loop, values] = msg.split(':');
    const parts = values.split(',');
    console.log(`MQTT PID for ${loop}:`, parts);
    if (!['inner', 'outer'].includes(loop)) return;
    pidValues[loop].push(parts);

    fs.appendFile('pid_log.txt', `${loop}: ${values}\n`, err => {
      if (err) console.error('Log write error:', err);
    });
  } else if (topic === 'robot/led' && msg === 'flash') {
    exec('sudo python3 flash_led.py', (err, stdout, stderr) => {
      if (err) console.error('LED Error:', stderr);
      else console.log('💡 LED flashed');
    });
  }
});

// Optional route to fetch stored PID values
app.get('/pid-values', (req, res) => {
  res.json(pidValues);
});

app.listen(PORT, () => {
  console.log(` Express server running at http://localhost:${PORT}`);
});
