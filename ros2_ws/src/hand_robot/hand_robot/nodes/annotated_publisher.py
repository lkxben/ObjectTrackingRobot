import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

class AnnotatedPublisher(Node):
    def __init__(self):
        super().__init__('annotated_publisher')

        self.bridge = CvBridge()
        self.latest_boxes = None

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.box_sub = self.create_subscription(
            Float32MultiArray,
            '/prompt/box',
            self.box_callback,
            10
        )

        self.pub = self.create_publisher(CompressedImage, '/camera/annotated/compressed', 10)

    def box_callback(self, msg):
        if msg.data and len(msg.data) == 4:
            self.latest_boxes = msg.data
        else:
            self.latest_boxes = None

    def image_callback(self, img_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            if self.latest_boxes:
                x, y, w, h = self.latest_boxes
                cv2.rectangle(frame,
                              (int(x), int(y)),
                              (int(x + w), int(y + h)),
                              (0, 255, 0),
                              2)

            frame = cv2.resize(frame, (320, 180))
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
            self.pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing annotated frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()