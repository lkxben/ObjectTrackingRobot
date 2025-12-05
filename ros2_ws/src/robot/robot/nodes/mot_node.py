import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np
import torch
from yolox.tracker.byte_tracker import BYTETracker
from robot_msgs.msg import Detection, DetectionArray, TurretState

class MOTNode(Node):
    def __init__(self):
        super().__init__('mot_node')
        self.det_sub = self.create_subscription(
            DetectionArray,
            '/detection/raw',
            self.detection_callback,
            10
        )
        self.input_sub = self.create_subscription(
            TurretState,
            '/turret/state',
            self.state_callback,
            10
        )

        self.mot_pub = self.create_publisher(DetectionArray, '/detection/mot', 10)
        self.overlay_pub = self.create_publisher(DetectionArray, '/detection/overlay', 10)
        class TrackerArgs:
            track_thresh = 0.1
            track_buffer = 30
            match_thresh = 0.8
            aspect_ratio_thresh = 1.6
            min_box_area = 10
            mot20 = False

        self.args = TrackerArgs()
        self.tracker = BYTETracker(self.args, frame_rate=30)
        self.frame_id = 0
        self.prev_prompt = None

        self.state = TurretState()
        self.state.mode = "IDLE"
        self.state.prompt = ""
        self.state.target_id = -1
        self.state.stamp = self.get_clock().now().to_msg()

        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.resolution = (320.0, 240.0)

        self.get_logger().info('MOT Setup - Complete')

    def info_callback(self, msg):
        if len(msg.data) >= 2:
            self.resolution = (msg.data[0], msg.data[1])

    def state_callback(self, msg):
        self.state = msg
        if self.prev_prompt != msg.prompt:
            self.prev_prompt = msg.prompt
            self.tracker = BYTETracker(self.args, frame_rate=30)
            self.frame_id = 0

    def detection_callback(self, msg):
        if not msg.detections:
            return
        
        boxes, scores = [], []
        for det in msg.detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            boxes.append([x1, y1, x2, y2])
            scores.append(det.confidence)

        if len(boxes) == 0:
            return

        output_results = np.concatenate([
            np.array(boxes),
            np.array(scores)[:, None]
        ], axis=1)

        output_results = torch.tensor(output_results, dtype=torch.float32)
        img_info = np.array([self.resolution[1], self.resolution[0]], dtype=np.float32)
        img_size = (self.resolution[1], self.resolution[0])
        online_targets = self.tracker.update(output_results, img_info, img_size)

        tracked_msg = DetectionArray()
        for t in online_targets:
            det = Detection()
            det.x1, det.y1, det.x2, det.y2 = t.tlbr
            det.confidence = float(t.score)
            det.track_id = t.track_id

            for orig_det in msg.detections:
                iou = self._bbox_iou(t.tlbr, [orig_det.x1, orig_det.y1, orig_det.x2, orig_det.y2])
                if iou > 0.5:
                    det.class_id = orig_det.class_id
                    det.class_name = orig_det.class_name
                    break
            tracked_msg.detections.append(det)

        if self.state.mode == 'TRACK' and self.state.target_id != -1:
            self.mot_pub.publish(tracked_msg)
        else:
            self.overlay_pub.publish(tracked_msg)
        self.frame_id += 1

    def _bbox_iou(self, boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        interArea = max(0, xB - xA) * max(0, yB - yA)
        boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
        boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
        iou = interArea / float(boxAArea + boxBArea - interArea + 1e-6)
        return iou

def main(args=None):
    rclpy.init(args=args)
    node = MOTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()