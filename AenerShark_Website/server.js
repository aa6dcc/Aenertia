const express = require('express');
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const app = express();

const PORT = 5000; 

let pidValues = {
  inner: [],
  outer: []
};

// Serve static files (HTML, CSS, JS)
app.use(express.static(path.join(__dirname)));

// Route for camera image
app.get('/snapshot', (req, res) => {
  exec('python3 capture_image.py', (err, stdout, stderr) => {
    if (err || stdout.trim() !== 'OK') {
      console.error('Camera error:', stderr);
      return res.status(500).send('Failed to capture image');
    }

    fs.readFile('snapshot.jpg', (err, data) => {
      if (err) return res.status(500).send('Image read error');
      res.writeHead(200, { 'Content-Type': 'image/jpeg' });
      res.end(data);
    });
  });
});

// Route for flashing LED
app.get('/flash-led', (req, res) => {
  exec('sudo python3 flash_led.py', (err, stdout, stderr) => {
    if (err) {
      console.error('LED Error:', stderr);
      return res.status(500).send('Failed to flash LED');
    }
    res.send('LED flashed!');
  });
});

// Route for receiving PID values
app.get('/pid', (req, res) => {
  const loop = req.query.loop;
  const values = req.query.values;

  console.log(`Set_PID for ${loop} loop: ${values}`);

  if (!['inner', 'outer'].includes(loop)) {
    return res.status(400).send('Invalid loop type');
  }

  const parts = values.split(' ');
  pidValues[loop].push(parts);

  fs.appendFile('pid_log.txt', `${loop}: ${values}\n`, (err) => {
    if (err) {
      console.error('Failed to write PID values:', err);
      return res.status(500).send('Failed to log PID values');
    }
    res.send(`PID values received for ${loop} loop`);
  });
});

// Route to fetch all stored PID values
app.get('/pid-values', (req, res) => {
  res.json(pidValues);
});

// Start server
app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
