from ultralytics import YOLO

model = YOLO("yolo11n-pose.pt")

results = model.train(data="hand-keypoints.yaml", epochs=10, imgsz=224, fraction=0.01)