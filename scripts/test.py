import cv2
import os
import numpy as np
import tensorflow as tf
from models.basic_cnn import build_basic_cnn, build_mobilenet_keypoints, build_efficientnet_keypoint_model

IMG_SIZE = 224
NUM_KEYPOINTS = 42

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "../models/checkpoints/best_weights.weights.h5")
# IMAGE_PATH = os.path.join(BASE_DIR, "../hand_middle.png")
IMAGE_PATH = os.path.join(BASE_DIR, "../dataset/training/color/00009.png")

# Load model
model = build_efficientnet_keypoint_model(input_shape=(IMG_SIZE, IMG_SIZE, 3), num_keypoints=NUM_KEYPOINTS)
model.load_weights(MODEL_PATH)
model.compile(optimizer='adam', loss='mse')

# Preprocess image
image = cv2.imread(IMAGE_PATH)
h, w, _ = image.shape
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image_resized = cv2.resize(image_rgb, (IMG_SIZE, IMG_SIZE))
image_resized = image_resized.astype("float32") / 255.0
input_tensor = np.expand_dims(image_resized, axis=0)

# Predict
pred = model.predict(input_tensor, verbose=0)[0]
pred = pred.reshape(NUM_KEYPOINTS, 3)

# Map to original image size
# x = (pred[:, 0]).astype(int)
# y = (pred[:, 1]).astype(int)
# v = pred[:, 2] > 0.5

image_disp = cv2.resize(image, (IMG_SIZE, IMG_SIZE))
for xi, yi, vi in zip((pred[:,0]*IMG_SIZE).astype(int), (pred[:,1]*IMG_SIZE).astype(int), pred[:,2]>0.8):
    if vi:
        cv2.circle(image_disp, (xi, yi), 3, (0,0,255), -1)
        
cv2.imshow("Hand Keypoints", image_disp)
cv2.waitKey(0)
cv2.destroyAllWindows()