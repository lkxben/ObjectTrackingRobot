import cv2
import requests
import numpy as np
from dotenv import load_dotenv
import os

load_dotenv()

url = os.environ.get("CAMERA_URL", "http://127.0.0.1:5002/video")

stream = requests.get(url, stream=True)
bytes_data = b''

for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow('MJPEG Stream', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()