import express from 'express';
import { WebSocketServer } from 'ws';
import http from 'http';
import path from 'path';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const app = express();
const server = http.createServer(app);
app.use(express.static(path.join(__dirname, '../frontend')));

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../frontend', 'index.html'));
});

// For frontend
const wssFrontend = new WebSocketServer({ server, path: '/rosbridge' });
let rosbridgeSocket = null;

wssFrontend.on('connection', (ws) => {
    console.log('Frontend connected');

    ws.on('message', (msg) => {
        if (rosbridgeSocket && rosbridgeSocket.readyState === ws.OPEN) {
            rosbridgeSocket.send(msg);
        }
    });

    ws.on('close', () => {
        console.log('Frontend disconnected');
    });
});

// For local rosbridge
const rosbridgeWss = new WebSocketServer({ server, path: '/localrosbridge' });

rosbridgeWss.on('upgrade', (request, socket, head) => {
    if (request.url === '/localrosbridge') {
        rosbridgeWss.handleUpgrade(request, socket, head, (ws) => {
            rosbridgeSocket = ws;
            console.log('Local ROSBridge connected');

            ws.on('message', (msg) => {
                wss.clients.forEach((client) => {
                    if (client.readyState === client.OPEN) {
                        client.send(msg);
                    }
                });
            });

            ws.on('close', () => {
                console.log('Local ROSBridge disconnected');
                rosbridgeSocket = null;
            });
        });
    } else {
        socket.destroy();
    }
});

const PORT = process.env.PORT || 443;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));