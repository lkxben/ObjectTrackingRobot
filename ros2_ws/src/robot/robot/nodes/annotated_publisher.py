import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge
from robot_msgs.msg import DetectionArray, TurretState
import cv2
import time

class AnnotatedPublisher(Node):
    def __init__(self):
        super().__init__('annotated_publisher')

        self.bridge = CvBridge()
        self.latest_detections = []
        self.last_det_time = 0.0
        self.det_timeout = 0.5
        self.last_hb_time = 0.0
        self.hb_timeout = 90.0
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.det_sub = self.create_subscription(
            DetectionArray,
            '/detection/overlay',
            self.detection_callback,
            10
        )

        self.state_sub = self.create_subscription(
            TurretState,
            '/turret/state',
            self.state_callback,
            10
        )

        self.hb_sub = self.create_subscription(
            Empty,
            '/viewer/heartbeat',
            self.hb_callback,
            10
        )

        self.annotated = True
        self.annotated_sub = self.create_subscription(
            Bool,
            '/annotated',
            self.annotated_callback,
            10
        )

        self.target_id = -1
        self.status = None
        self.pub = self.create_publisher(CompressedImage, '/camera/annotated/compressed', 10)
        self.get_logger().info('Annotated Publisher Started')

    def annotated_callback(self, msg):
        self.annotated = msg.data

    def hb_callback(self, msg):
        self.last_hb_time = time.time()

    def detection_callback(self, msg):
        if self.status == 'TRACKING':
            if len(msg.detections) > 0:
                for det in msg.detections:
                    if det.track_id == self.target_id:
                        da = DetectionArray()
                        da.detections.append(det)
                        self.latest_detections = da.detections
                        self.last_det_time = time.time()
                        break
            else:
                self.latest_detections = []
        else:
            if len(msg.detections) > 0:
                self.latest_detections = msg.detections
                self.last_det_time = time.time()
            else:
                self.latest_detections = []

    def state_callback(self, msg):
        self.status = msg.status
        self.target_id = msg.target_id

    def image_callback(self, img_msg):
        try:
            now = time.time()

            if now - self.last_hb_time > self.hb_timeout:
                return

            if now - self.last_det_time > self.det_timeout:
                self.latest_detections = []

            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            if self.annotated:
                for det in self.latest_detections:
                    x1, y1, x2, y2 = int(det.x1), int(det.y1), int(det.x2), int(det.y2)
                    class_name = det.class_name
                    conf = det.confidence
                    track_id = det.track_id

                    # Draw box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    # Text label
                    label = f"{class_name} {conf:.2f} {track_id}"
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 5),
                        cv2.FONT_ITALIC,
                        0.5,
                        (0, 0, 255),
                        1,
                        cv2.LINE_8
                    )

            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            self.pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing annotated frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()