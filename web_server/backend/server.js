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

const wss = new WebSocketServer({ server, path: '/rosbridge' });
let rosbridgeSocket = null;
const frontends = new Set()

wss.on('connection', (ws, request) => {
  const url = new URL(request.url, `http://${request.headers.host}`)
  const clientType = url.searchParams.get('client')

  if (clientType === 'localrosbridge') {
    rosbridgeSocket = ws
    console.log('Local ROSBridge connected')

    ws.on('message', msg => {
        console.log('Forwarding message type:', typeof msg)
        const forwardMsg = msg instanceof Buffer ? msg.toString() : msg
        for (const client of frontends) {
            if (client.readyState === client.OPEN) client.send(forwardMsg)
        }
    })
    ws.on('close', () => {
      console.log('Local ROSBridge disconnected')
      rosbridgeSocket = null
    })
  } else if (clientType === 'frontend') {
    frontends.add(ws)
    console.log('Frontend connected')

    ws.on('message', msg => {
      if (rosbridgeSocket && rosbridgeSocket.readyState === rosbridgeSocket.OPEN)
        rosbridgeSocket.send(msg)
    })
    ws.on('close', () => {
      console.log('Frontend disconnected')
      frontends.delete(ws)
    })
  } else {
    console.log('Unknown client tried to connect, closing')
    ws.close()
  }
});

const PORT = process.env.PORT || 443;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));