import json
import os
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def freihand_to_yolo(xyz_json_path, K_json_path, images_dir, output_dir, padding = 0.1):
    os.makedirs(output_dir, exist_ok=True)
    
    with open(xyz_json_path, 'r') as f:
        xyz_data = json.load(f)
    
    with open(K_json_path, 'r') as f:
        K_data = json.load(f)
    
    for idx, joints_3d in enumerate(xyz_data):
        K = np.array(K_data[idx])  # 3x3 camera intrinsic matrix
        joints_3d = np.array(joints_3d)  # shape (21, 3)
        
        # Project 3D -> 2D
        joints_h = np.concatenate([joints_3d, np.ones((21,1))], axis=1).T  # (4,21)
        uv_h = K @ joints_h[:3]  # (3,21)
        uv = (uv_h[:2] / uv_h[2]).T  # (21,2) pixel coordinates
        
        # Compute bounding box with padding around keypoints
        x_min, y_min = uv.min(axis=0)
        x_max, y_max = uv.max(axis=0)
        box_w = (x_max - x_min) * (1 + padding)
        box_h = (y_max - y_min) * (1 + padding)
        x_c = x_min + (x_max - x_min)/2
        y_c = y_min + (y_max - y_min)/2

        # Get image size
        img_path = os.path.join(images_dir, f'{idx:08d}.jpg')  # 8 digits
        from PIL import Image
        with Image.open(img_path) as im:
            img_w, img_h = im.size
        
        # Normalize for YOLO format
        x_c /= img_w
        y_c /= img_h
        box_w /= img_w
        box_h /= img_h
        
        # Prepare keypoints in YOLO normalized format (x1,y1,...,x21,y21)
        keypoints_norm = []
        for (x, y) in uv:
            keypoints_norm.extend([x/img_w, y/img_h, 2])  # 2=visible
        
        # YOLO label: class_id x_center y_center width height kpt1_x kpt1_y v1 ...
        yolo_line = f"0 {x_c} {y_c} {box_w} {box_h} " + " ".join(map(str,keypoints_norm))
        
        # Write to file
        out_txt = os.path.join(output_dir, f'{idx:08d}.txt')
        with open(out_txt, 'w') as f:
            f.write(yolo_line + '\n')


xyz_json_path = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'raw', 'training_xyz.json')
K_json_path = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'raw', 'training_K.json')
images_dir = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'images', 'train')
output_dir = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'labels', 'train')
freihand_to_yolo(
    xyz_json_path,
    K_json_path,
    images_dir,
    output_dir
)

xyz_json_path = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'raw', 'evaluation_xyz.json')
K_json_path = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'raw', 'evaluation_K.json')
images_dir = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'images', 'val')
output_dir = os.path.join(BASE_DIR, '..', 'datasets', 'friehand', 'labels', 'val')
freihand_to_yolo(
    xyz_json_path,
    K_json_path,
    images_dir,
    output_dir
)