import os
import numpy as np
import tensorflow as tf
from models.basic_cnn import build_basic_cnn
from scripts.data_utils import create_tf_dataset, load_annotations

IMG_SIZE = 224
NUM_KEYPOINTS = 42
BATCH_SIZE = 32
PIXEL_THRESHOLD = 5
THRESHOLD_NORM = PIXEL_THRESHOLD / 320.0

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "../models/checkpoints/best_weights.weights.h5")
EVAL_DIR = os.path.join(BASE_DIR, "../dataset/evaluation")
IMAGE_DIR = os.path.join(EVAL_DIR, "color")
ANNOTATION_FILE = os.path.join(EVAL_DIR, "anno_evaluation.pickle")

model = build_basic_cnn(input_shape=(IMG_SIZE, IMG_SIZE, 3), num_keypoints=NUM_KEYPOINTS)
model.load_weights(MODEL_PATH)
model.compile(optimizer='adam', loss='mse')

val_annotations = load_annotations(os.path.join(EVAL_DIR, "anno_evaluation.pickle"))
val_gen = create_tf_dataset(os.path.join(EVAL_DIR, "color"), val_annotations, batch_size=BATCH_SIZE, shuffle=False)

def keypoint_accuracy(y_true, y_pred, threshold=THRESHOLD_NORM):
    """
    Inputs are normalized (0–1). Convert back to pixel space (0–320).
    y_true, y_pred: shape (num_samples, num_keypoints * 3)
    """
    xy_true = y_true[:, :, :2]
    xy_pred = y_pred[:, :, :2]
    dists = np.linalg.norm(xy_true - xy_pred, axis=-1)
    correct = dists < threshold
    return correct.mean() * 100

y_true_all, y_pred_all = [], []

for images, keypoints in val_gen:
    preds = model.predict(images, verbose=0)
    y_true_all.append(keypoints.numpy())
    y_pred_all.append(preds)

y_true_all = np.concatenate(y_true_all, axis=0).reshape(-1, NUM_KEYPOINTS, 3)
y_pred_all = np.concatenate(y_pred_all, axis=0).reshape(-1, NUM_KEYPOINTS, 3)

acc = keypoint_accuracy(y_true_all, y_pred_all)
print(f"\n Percentage of keypoints within {PIXEL_THRESHOLD} pixels: {acc:.2f}%")