import rclpy
from rclpy.node import Node
from robot_msgs.msg import DetectionArray
from std_msgs.msg import Float32MultiArray
import numpy as np
import torch
from yolox.tracker.byte_tracker import BYTETracker, STrack
from robot_msgs.msg import Detection, DetectionArray

class ByteTrackNode(Node):
    def __init__(self):
        super().__init__('bytetrack_node')
        self.det_sub = self.create_subscription(
            DetectionArray,
            '/detection',
            self.detection_callback,
            10
        )
        self.track_pub = self.create_publisher(DetectionArray, '/detection/tracked', 10)
        class TrackerArgs:
            track_thresh = 0.1
            track_buffer = 30
            match_thresh = 0.8
            aspect_ratio_thresh = 1.6
            min_box_area = 10
            mot20 = False

        args = TrackerArgs()
        self.tracker = BYTETracker(args, frame_rate=30)
        self.frame_id = 0

        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)

        self.get_logger().info('ByteTrack Setup - Complete')

    def info_callback(self, msg):
        self.resolution = (msg.data[0], msg.data[1])

    def detection_callback(self, msg):
        if not msg.detections:
            return
            
        # # filter duplicate detections by confidence
        # filtered_detections = []
        # used = set()
        # for i, det1 in enumerate(msg.detections):
        #     if i in used:
        #         continue
        #     group = [det1]
        #     for j, det2 in enumerate(msg.detections):
        #         if j == i or j in used:
        #             continue
        #         if self._bbox_iou([det1.x1, det1.y1, det1.x2, det1.y2],
        #                         [det2.x1, det2.y1, det2.x2, det2.y2]) > 0.5:
        #             group.append(det2)
        #             used.add(j)
        #     best = max(group, key=lambda d: d.confidence)
        #     filtered_detections.append(best)
        
        boxes, scores, cls_ids = [], [], []
        for det in msg.detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2
            tlwh = STrack.tlbr_to_tlwh([x1, y1, x2, y2])
            boxes.append(tlwh)
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

            # for orig_det in filtered_detections:
            #     iou = self._bbox_iou(t.tlbr, [orig_det.x1, orig_det.y1, orig_det.x2, orig_det.y2])
            #     if iou > 0.5:
            #         det.class_id = orig_det.class_id
            #         det.class_name = orig_det.class_name
            #         break
            tracked_msg.detections.append(det)

        self.track_pub.publish(tracked_msg)
        self.frame_id += 1
        self.get_logger().info(str(tracked_msg))

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
    node = ByteTrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()