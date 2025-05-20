// const express = require('express');
// const path = require('path');
// const app = express();
// const PORT = 3000;

// // Serve all static files from the current directory
// app.use(express.static(path.join(__dirname)));

// // Start the server
// app.listen(PORT, () => {
//   console.log(`Server running at http://localhost:${PORT}`);
// });
// server.js
const express = require('express');
const { exec } = require('child_process');
const fs = require('fs');
const app = express();

app.get('/snapshot', (req, res) => {
  exec('python3 capture_image.py', (err, stdout, stderr) => {
    if (err || stdout.trim() !== 'OK') {
      console.error('Python error:', stderr);
      return res.status(500).send('Failed to capture image');
    }

    fs.readFile('snapshot.jpg', (err, data) => {
      if (err) return res.status(500).send('Image read error');
      res.writeHead(200, { 'Content-Type': 'image/jpeg' });
      res.end(data);
    });
  });
});

app.listen(5000, () => console.log('Server running on http://<pi_ip>:5000'));

