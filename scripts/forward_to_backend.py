import websocket
import threading
import json

LOCAL_ROSBRIDGE = "ws://localhost:9090"
BACKEND_WSS = "wss://handguesturerobot.onrender.com/rosbridge?client=localrosbridge"

def forward_ros_to_backend():
    ws_local = websocket.WebSocket()
    ws_local.connect(LOCAL_ROSBRIDGE)

    ws_backend = websocket.WebSocket()
    ws_backend.connect(BACKEND_WSS)

    def receive_local():
        while True:
            msg = ws_local.recv()
            ws_backend.send(msg)

    def receive_backend():
        while True:
            msg = ws_backend.recv()
            ws_local.send(msg)

    threading.Thread(target=receive_local).start()
    threading.Thread(target=receive_backend).start()

forward_ros_to_backend()