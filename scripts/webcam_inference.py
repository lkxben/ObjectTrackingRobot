import cv2
import os
import numpy as np
import tensorflow as tf
from ultralytics import YOLO

IMG_SIZE = 224
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Load model
model = YOLO("weights/no-fh3.pt")

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Run YOLO inference
    results = model.predict(source=frame, imgsz=224, conf=0.5, verbose=False)

    for result in results:
        if result.keypoints is None or result.keypoints.data.shape[0] == 0:
            continue  # No detection, move to next frame

        keypoints = result.keypoints.data[0].cpu().numpy()
        for kp in keypoints:
            x, y = int(kp[0]), int(kp[1])
            conf = kp[2] if len(kp) > 2 else 1.0
            if conf > 0.5:
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

    cv2.imshow("Hand Keypoints", frame)

    # Exit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()