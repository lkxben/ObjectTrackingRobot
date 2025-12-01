import websocket
import threading
import json
import time
import queue

LOCAL_ROSBRIDGE = "ws://localhost:9090"
BACKEND_WSS = "wss://handguesturerobot.onrender.com/rosbridge?client=localrosbridge"

BACKEND_TOPICS = [
    {"topic": "/prompt/input", "type": "std_msgs/String"}
]

backend_queue = queue.Queue()

def ros_to_backend():
    ws_local = websocket.WebSocket()
    ws_local.connect(LOCAL_ROSBRIDGE)

    ws_backend = websocket.WebSocket()
    ws_backend.connect(BACKEND_WSS)

    sub_msg = {
        "op": "subscribe",
        "topic": "/camera/annotated/compressed",
        "type": "sensor_msgs/CompressedImage",
        "throttle_rate": 0
    }
    ws_local.send(json.dumps(sub_msg))

    for t in BACKEND_TOPICS:
        adv_msg = {
            "op": "advertise",
            "topic": t["topic"],
            "type": t["type"]
        }
        ws_local.send(json.dumps(adv_msg))

    def receive_backend():
        while True:
            try:
                msg = ws_backend.recv()
                backend_queue.put(msg)
            except websocket.WebSocketConnectionClosedException:
                time.sleep(1)
            except Exception:
                time.sleep(0.01)

    def process_backend_queue():
        while True:
            try:
                msg = backend_queue.get(timeout=0.1)
                ws_local.send(msg)
                backend_queue.task_done()
            except queue.Empty:
                continue
            except websocket.WebSocketConnectionClosedException:
                time.sleep(1)
            except Exception:
                time.sleep(0.01)

    def receive_local():
        while True:
            try:
                msg = ws_local.recv()
                ws_backend.send(msg)
            except websocket.WebSocketConnectionClosedException:
                time.sleep(1)
            except Exception:
                time.sleep(0.01)

    threading.Thread(target=receive_backend, daemon=True).start()
    threading.Thread(target=process_backend_queue, daemon=True).start()
    threading.Thread(target=receive_local, daemon=True).start()

    # Keep main thread alive
    while True:
        time.sleep(1)

ros_to_backend()