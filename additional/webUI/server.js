const express = require('express');
const cors = require('cors');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.static('public'));

// Serve the main HTML file
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// API endpoint for logging face detection events (optional)
app.post('/api/log-detection', (req, res) => {
  const { event, timestamp } = req.body;
  console.log(`Face Detection Event: ${event} at ${timestamp}`);
  res.json({ success: true });
});

// Start server
app.listen(PORT, () => {
  console.log(`Libot Interface running on http://localhost:${PORT}`);
  console.log('Press Ctrl+C to stop the server');
});