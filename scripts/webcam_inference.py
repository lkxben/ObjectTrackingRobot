import cv2
import os
import numpy as np
import tensorflow as tf
from models.basic_cnn import build_basic_cnn

IMG_SIZE = 224
NUM_KEYPOINTS = 42
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "../models/checkpoints/best_weights.weights.h5")

# Load model
model = build_basic_cnn(input_shape=(IMG_SIZE, IMG_SIZE, 3), num_keypoints=NUM_KEYPOINTS)
model.load_weights(MODEL_PATH)
model.compile(optimizer='adam', loss='mse')

def preprocess_frame(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_resized = cv2.resize(frame_rgb, (IMG_SIZE, IMG_SIZE))
    frame_resized = frame_resized.astype("float32") / 255.0
    return np.expand_dims(frame_resized, axis=0)

def postprocess_keypoints(pred, original_size):
    pred = pred.reshape(NUM_KEYPOINTS, 3)
    # xy = pred[:, :2].copy()
    # xy[:, 0] *= original_size[0]
    # xy[:, 1] *= original_size[1]
    x = (pred[:, 0] / 320.0 * original_size[0]).astype(int)
    y = (pred[:, 1] / 320.0 * original_size[1]).astype(int)
    v = pred[:, 2] > 0
    return np.stack([x, y, v], axis=1)


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    h, w, _ = frame.shape
    input_frame = preprocess_frame(frame)
    pred = model.predict(input_frame, verbose=0)
    # print("Raw prediction shape:", pred.shape)
    # print("Raw prediction values:", pred)
    keypoints = postprocess_keypoints(pred[0], (w, h))

    # v = pred[:, 2]
    # for (x, y), vis in zip(keypoints, v):
    #     # if vis > 0.5:  # threshold
    #     cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
    for x, y, vis in keypoints:
        if vis:
            cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)

    cv2.imshow("Hand Keypoints", frame)
    
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()