import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from robot_msgs.msg import DetectionArray
import cv2
import time

class AnnotatedPublisher(Node):
    def __init__(self):
        super().__init__('annotated_publisher')

        self.bridge = CvBridge()
        self.latest_detections = []
        self.last_det_time = 0.0
        self.det_timeout = 0.5

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.det_sub = self.create_subscription(
            DetectionArray,
            '/detection/tracked',
            self.detection_callback,
            10
        )

        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.resolution = (320.0, 240.0)

        self.pub = self.create_publisher(CompressedImage, '/camera/annotated/compressed', 10)
        self.get_logger().info('Annotated Publisher Setup - Complete')

    def info_callback(self, msg):
        self.resolution = (msg.data[0], msg.data[1])

    def detection_callback(self, msg: DetectionArray):
        if len(msg.detections) > 0:
            self.latest_detections = msg.detections
            self.last_det_time = time.time()
        else:
            self.latest_detections = []

    def image_callback(self, img_msg):
        try:
            now = time.time()

            if now - self.last_det_time > self.det_timeout:
                self.latest_detections = []

            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

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

            # frame = cv2.resize(frame, (int(self.resolution[0] * 0.9), int(self.resolution[1] * 0.9)))

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