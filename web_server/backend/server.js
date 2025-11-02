// const express = require('express');
// const path = require('path');
// const WebSocket = require('ws');

// const app = express();
// const PORT = process.env.WEBSOCKET_PORT || 8080;

// app.use(express.static(path.join(__dirname, '../frontend')));

// const server = app.listen(PORT, () => {
//     console.log(`HTTP server running on port ${PORT}`);
// });

// // WebSocket setup
// const wss = new WebSocket.Server({ server });

// wss.on('connection', (ws) => {
//     console.log('WebSocket client connected');
//     ws.on('message', (message) => {
//         const msgStr = message.toString();
//         for (const client of wss.clients) {
//             if (client.readyState === WebSocket.OPEN) {
//                 client.send(msgStr);
//             }
//         }
//     });
// });

import express from 'express';
import fs from 'fs';
import path from 'path';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3000;

// Serve static assets (css, js, images) if any
app.use('/static', express.static(path.join(__dirname, 'static')));

// Serve HTML and inject ROSBRIDGE_URL
app.get('/', (req, res) => {
  const htmlPath = path.join(__dirname, 'index.html');
  let html = fs.readFileSync(htmlPath, 'utf-8');

  // Replace placeholder with env variable
  html = html.replace('{{ROSBRIDGE_URL}}', process.env.ROSBRIDGE_URL || 'ws://localhost:9090');

  res.send(html);
});

app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});