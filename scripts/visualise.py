import os
import pickle
import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "../dataset/training/color")
ANNOTATION_FILE = os.path.join(BASE_DIR, "../dataset/training/anno_training.pickle")

# Load annotations
with open(ANNOTATION_FILE, "rb") as f:
    annotations = pickle.load(f)

# Pick a sample image
sample_idx = "00010"  # change as needed
image_path = os.path.join(IMAGE_DIR, f"{sample_idx}.png")
image = cv2.imread(image_path)

keypoints = annotations[int(sample_idx)]["uv_vis"]
h, w, _ = image.shape

# Map normalized keypoints to pixel coordinates
x = (np.array(keypoints)[:,0]).astype(int)
y = (np.array(keypoints)[:,1]).astype(int)
v = np.array(keypoints)[:,2] > 0.5

# Draw keypoints
for xi, yi, vi in zip(x, y, v):
    cv2.circle(image, (xi, yi), 3, (0,0,255), -1)

cv2.imshow("Ground Truth Keypoints", image)
cv2.waitKey(0)
cv2.destroyAllWindows()