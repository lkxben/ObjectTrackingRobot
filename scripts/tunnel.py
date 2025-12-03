import websocket
import threading
import json
import time
import queue
from websocket import ABNF, WebSocketConnectionClosedException

LOCAL = "ws://localhost:9090"
BACKEND = "wss://handguesturerobot.onrender.com/rosbridge?client=localrosbridge"

BACKEND_TOPICS = [
    {"topic": "/prompt/input", "type": "std_msgs/String"},
    {"topic": "/tracked/id", "type": "std_msgs/Int32" }
]

backend_to_local_queue = queue.Queue(maxsize=200)


def connect(url):
    while True:
        try:
            return websocket.create_connection(
                url, ping_interval=20, ping_timeout=10, compression=None
            )
        except:
            time.sleep(1)


def advertise_topics(ws_local):
    for t in BACKEND_TOPICS:
        adv = {
            "op": "advertise",
            "topic": t["topic"],
            "type": t["type"],
            "latch": False,
            "queue_size": 10
        }
        ws_local.send(json.dumps(adv))
    time.sleep(0.1)  # allow rosbridge to set up publishers


def start_tunnel():
    while True:
        ws_local = connect(LOCAL)
        ws_backend = connect(BACKEND)

        sub = {
            "op": "subscribe",
            "topic": "/camera/annotated/compressed",
            "type": "sensor_msgs/CompressedImage",
            "throttle_rate": 0,
            "fragment_size": 64000
        }
        ws_local.send(json.dumps(sub))

        advertise_topics(ws_local)

        # Backend → Local (RELIABLE, NEVER DROP)
        def recv_backend():
            while True:
                try:
                    msg = ws_backend.recv()
                    backend_to_local_queue.put(msg, timeout=1)
                except:
                    break  # reconnect outer loop

        def send_to_local():
            while True:
                try:
                    msg = backend_to_local_queue.get(timeout=1)
                    ws_local.send(msg)
                    backend_to_local_queue.task_done()
                except queue.Empty:
                    continue
                except:
                    break

        # Local → Backend (UNRELIABLE, DROP WHEN SLOW)
        def recv_local():
            while True:
                try:
                    frame = ws_local.recv_frame()
                    if frame.opcode == ABNF.OPCODE_TEXT:
                        try:
                            ws_backend.send(frame.data.decode("utf-8"))
                        except WebSocketConnectionClosedException:
                            break
                        except:
                            time.sleep(0.01)
                            continue
                except WebSocketConnectionClosedException:
                    break
                except:
                    time.sleep(0.01)
                    continue

        threading.Thread(target=recv_backend, daemon=True).start()
        threading.Thread(target=send_to_local, daemon=True).start()
        threading.Thread(target=recv_local, daemon=True).start()

        # stay alive until a connection breaks
        while True:
            try:
                ws_local.ping()
                ws_backend.ping()
                time.sleep(5)
            except:
                break  # reconnect loop continues


start_tunnel()