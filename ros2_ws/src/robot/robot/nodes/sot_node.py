import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from robot_msgs.msg import Detection, DetectionArray
from cv_bridge import CvBridge
import cv2
import sys
import torch
torch.Tensor.cuda = lambda self, device=None: self
import numpy as np

sys.path.append('/root/DaSiamRPN/code')
from net import SiamRPNBIG
from run_SiamRPN import SiamRPN_init, SiamRPN_track
from utils import cxy_wh_2_rect

class SOTNode(Node):
    def __init__(self):
        super().__init__('sot_node')

        self.bridge = CvBridge()
        self.active_target_id = None
        self.pending_bbox = None
        self.tracker_state = None
        self.target_class = None

        self.net = SiamRPNBIG()
        self.net.load_state_dict(torch.load('/root/DaSiamRPN/code/SiamRPNBIG.model', map_location=torch.device('cpu'), weights_only=False))
        self.net.eval()

        self.det_sub = self.create_subscription(
            DetectionArray, '/detection/mot', self.detection_callback, 10)
        self.target_id_sub = self.create_subscription(
            Int32, '/tracked/id', self.target_id_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.track_pub = self.create_publisher(
            DetectionArray, '/detection/sot', 10)

        self.get_logger().info("SOT (DaSiamRPN) Node Initialized")

    def target_id_callback(self, msg):
        self.active_target_id = msg.data
        self.tracker_state = None
        self.target_class = None
        self.pending_bbox = None
        self.get_logger().info(f"Selected object ID: {msg.data}")

    def detection_callback(self, msg):
        if self.active_target_id is None or self.tracker_state is not None:
            return

        for det in msg.detections:
            if det.track_id == self.active_target_id:
                cx = (det.x1 + det.x2) / 2
                cy = (det.y1 + det.y2) / 2
                w = det.x2 - det.x1
                h = det.y2 - det.y1
                self.pending_bbox = np.array([cx, cy, w, h])
                self.target_class = det.class_name
                self.get_logger().info(f"Pending bbox for tracker ID {det.track_id}: {self.pending_bbox}")
                break

    def image_callback(self, msg):
        if self.active_target_id is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.pending_bbox is not None and self.tracker_state is None:
            # Initialize tracker
            cx, cy, w, h = self.pending_bbox
            target_pos = np.array([cx, cy])
            target_sz = np.array([w, h])
            self.tracker_state = SiamRPN_init(frame, target_pos, target_sz, self.net)
            self.pending_bbox = None
            self.get_logger().info(f"Tracker initialized for ID {self.active_target_id}")
            return

        if self.tracker_state is not None:
            # Track object in new frame
            self.tracker_state = SiamRPN_track(self.tracker_state, frame)
            bbox = cxy_wh_2_rect(self.tracker_state['target_pos'], self.tracker_state['target_sz'])
            x, y, w, h = bbox.astype(float) 

            det_msg = Detection()
            det_msg.track_id = self.active_target_id
            det_msg.x1 = x
            det_msg.y1 = y
            det_msg.x2 = x + w
            det_msg.y2 = y + h
            det_msg.class_name = self.target_class

            arr_msg = DetectionArray()
            arr_msg.detections.append(det_msg)
            self.track_pub.publish(arr_msg)
            # self.get_logger().info(str(arr_msg))

def main(args=None):
    rclpy.init(args=args)
    node = SOTNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()