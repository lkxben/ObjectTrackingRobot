import cv2
import os
import numpy as np

def visualize_yolo_pose_numbered_large(image_path, label_path, scale=2):
    img = cv2.imread(image_path)
    h, w, _ = img.shape

    # Scale up image
    img = cv2.resize(img, (w*scale, h*scale))
    
    with open(label_path, 'r') as f:
        line = f.readline().strip().split()

    class_id = int(line[0])
    x_c, y_c, box_w, box_h = map(float, line[1:5])
    kpt_data = list(map(float, line[5:]))

    # convert normalized bbox to pixels (scaled)
    x1 = int((x_c - box_w/2) * w * scale)
    y1 = int((y_c - box_h/2) * h * scale)
    x2 = int((x_c + box_w/2) * w * scale)
    y2 = int((y_c + box_h/2) * h * scale)

    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    for i in range(21):
        kx = int(kpt_data[i*3] * w * scale)
        ky = int(kpt_data[i*3 + 1] * h * scale)
        v = kpt_data[i*3 + 2]
        if v > 0:
            cv2.circle(img, (kx, ky), 4*scale//2, (0, 0, 255), -1)
            cv2.putText(img, str(i), (kx+5, ky-5), cv2.FONT_HERSHEY_SIMPLEX, 0.8*scale/2, (255, 255, 0), 2)

    cv2.imshow("YOLO-Pose Annotation", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(BASE_DIR, "../datasets/friehand/images/train/00000011.jpg")
label_path = os.path.join(BASE_DIR, "../datasets/friehand/labels/train/00000011.txt")
visualize_yolo_pose_numbered_large(image_path, label_path, scale=10)