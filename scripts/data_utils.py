import os
import pickle
import numpy as np
import cv2
from tensorflow.keras.utils import Sequence

IMG_SIZE = 224

def load_annotations(pickle_path):
    with open(pickle_path, "rb") as f:
        annotations = pickle.load(f)
    return annotations

def preprocess_image(image_path):
    img = cv2.imread(image_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
    img = img.astype("float32") / 255.0
    return img

def preprocess_keypoints(keypoints, original_size=(320, 320)):
    keypoints = np.array(keypoints)
    keypoints[:, 0] /= original_size[0]
    keypoints[:, 1] /= original_size[1]
    return keypoints

class RHDDataGenerator(Sequence):
    def __init__(self, image_dir, annotations, batch_size=32, shuffle=True):
        self.image_dir = image_dir
        self.annotations = annotations
        self.keys = list(annotations.keys())
        self.batch_size = batch_size
        self.shuffle = shuffle
        self.on_epoch_end()
    
    def __len__(self):
        return len(self.keys) // self.batch_size
    
    def __getitem__(self, idx):
        batch_keys = self.keys[idx * self.batch_size : (idx + 1) * self.batch_size]
        images, keypoints = [], []
        for k in batch_keys:
            img_path = os.path.join(self.image_dir, f"{k:05d}.png")
            img = preprocess_image(img_path)
            kp = preprocess_keypoints(self.annotations[k]["uv_vis"][:, :2])
            images.append(img)
            keypoints.append(kp.flatten())
        return np.array(images), np.array(keypoints)
    
    def on_epoch_end(self):
        if self.shuffle:
            np.random.shuffle(self.keys)