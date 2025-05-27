// Express server for handling hardware control and PID data logging
// - Serves static frontend files
// - Captures images from a connected camera
// - Controls an LED
// - Receives and logs PID controller values

const express = require('express');
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');
const app = express();

const PORT = 5000;

let pidValues = {
  inner: [],
  outer: []
}; // Object to store PID values for 'inner' and 'outer' control loops

// Serve static files (HTML, CSS, JS)
app.use(express.static(path.join(__dirname)));

// Route for camera image
// Capture image from camera via Python script and send as JPEG response
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
// Run Python script to flash an LED (requires sudo privileges)
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
// Receive and log PID values for specified control loop ('inner' or 'outer')
app.get('/pid', (req, res) => {
  const loop = req.query.loop;
  const values = req.query.values;

  if (!['inner', 'outer'].includes(loop)) { // Validate loop type ('inner' or 'outer')
    return res.status(400).send('Invalid loop type');
  }

  const parts = values.split(','); // Parse comma-separated PID values from query string
  pidValues[loop].push(parts);

  const pidString = `SET_PID_${loop} ${parts.join(' ')}`; // Desired format
  console.log(pidString);

  // Format and log the PID values to file
  fs.appendFile('pid_log.txt', `${pidString}\n`, (err) => {
    if (err) {
      console.error('Failed to write PID values:', err);
      return res.status(500).send('Failed to log PID values');
    }
    res.send(`PID values received for ${loop} loop`);
  });
});

// Route to fetch all stored PID values
// Return all collected PID values as JSON
app.get('/pid-values', (req, res) => {
  res.json(pidValues);
});

// Start server
// Start server on specified port and log the address
app.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
