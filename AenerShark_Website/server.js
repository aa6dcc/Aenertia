const express = require('express');
const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 6000;

// Serve static files (index.html, assets, etc.)
app.use(express.static(path.join(__dirname)));

// Route to flash LED
app.get('/flash-led', (req, res) => {
  exec('sudo python3 flash_led.py', (err, stdout, stderr) => {
    if (err) {
      console.error('LED Error:', stderr);
      return res.status(500).send('Failed to flash LED');
    }
    res.send('LED flashed!');
  });
});

// Route to take and return snapshot
app.get('/snapshot', (req, res) => {
  exec('python3 capture_image.py', (err, stdout, stderr) => {
    if (err || !stdout.includes("OK")) {
      console.error("Camera error:", stderr);
      return res.status(500).send("Failed to capture image");
    }

    fs.readFile('snapshot.jpg', (err, data) => {
      if (err) {
        console.error("Image read error");
        return res.status(500).send("Error reading image");
      }

      res.writeHead(200, { 'Content-Type': 'image/jpeg' });
      res.end(data);
    });
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});
