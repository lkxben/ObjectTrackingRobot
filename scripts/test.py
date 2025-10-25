import os
from ultralytics import YOLO

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_PATH = os.path.join(BASE_DIR, "../hand_middle.png")

model = YOLO("runs/pose/train/weights/best.pt")
model.predict(IMAGE_PATH, save=True, imgsz=320)