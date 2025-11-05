import express from 'express';
import path from 'path';
import { fileURLToPath } from 'url';
import WebSocket from 'ws';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const app = express();
const PORT = process.env.WEBSOCKET_PORT || 8080;

app.use(express.static(path.join(__dirname, '../frontend')));

const server = app.listen(PORT, () => {
    console.log(`HTTP server running on port ${PORT}`);
});

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
