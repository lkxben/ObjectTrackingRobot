import express from 'express';
import { WebSocketServer } from 'ws';
import http from 'http';

const app = express();
const server = http.createServer(app);

const wss = new WebSocketServer({ server, path: '/rosbridge' });

// For frontend
let rosbridgeSocket = null;

wss.on('connection', (ws) => {
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
const rosbridgeWss = new WebSocketServer({ noServer: true });

server.on('upgrade', (request, socket, head) => {
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