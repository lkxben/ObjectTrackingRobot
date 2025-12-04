import rclpy
from rclpy.node import Node
from robot_msgs.msg import DetectionArray
from std_msgs.msg import Float32MultiArray, Int32
from robot_msgs.msg import Detection, DetectionArray
import torch
from fear_xs.tracker.tracker import Tracker
from fear_xs.tracker.utils import Detection as FEARDetection

class SOTNode(Node):
    def __init__(self):
        super().__init__('sot_node')
        self.det_sub = self.create_subscription(
            DetectionArray,
            '/detection/tracked',
            self.detection_callback,
            10
        )
        self.trackId_sub = self.create_subscription(
            Int32,
            '/tracked/id',
            self.trackId_callback,
            10
        )
        self.trackId = -1

        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.resolution = (320.0, 240.0)

        # self.track_pub = self.create_publisher(DetectionArray, '/detection/tracked', 10)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.tracker = Tracker(device=self.device)
        self.active_track_id = None

        self.get_logger().info('SOT Setup - Complete')

    def info_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.resolution = (msg.data[0], msg.data[1])

    def trackId_callback(self, msg):
        self.get_logger().info(f"Received track ID: {msg.data}")
        self.trackId = msg.data

    def detection_callback(self, msg: DetectionArray):
        fear_dets = []
        for det in msg.detections:
            fear_det = FEARDetection(
                bbox=[det.x, det.y, det.width, det.height],
                score=det.confidence,
                feature=None
            )
            fear_dets.append(fear_det)

        tracked_objects = self.tracker.update(fear_dets)

        tracked_msg = DetectionArray()
        tracked_msg.header.stamp = msg.header.stamp
        tracked_msg.header.frame_id = msg.header.frame_id

        for obj in tracked_objects:
            if self.active_track_id is not None and obj.track_id != self.active_track_id:
                continue
            det = Detection()
            det.id = obj.track_id
            det.x, det.y, det.width, det.height = obj.bbox
            det.confidence = obj.score
            tracked_msg.detections.append(det)

        self.track_pub.publish(tracked_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SOTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()