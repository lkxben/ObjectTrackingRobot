import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class AnnotatedPublisher(Node):
    def __init__(self):
        super().__init__('annotated_publisher')
        self.get_logger().info('Started AnnotatedPublisher node')

        self.bridge = CvBridge()

        self.image_sub = Subscriber(self, Image, '/camera/image_raw')
        self.box_sub = Subscriber(self, Float32MultiArray, '/hand/box')

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.box_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.synced_callback)

        self.pub = self.create_publisher(CompressedImage, '/camera/annotated/compressed', 10)

    def synced_callback(self, img_msg, box_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            if box_msg.data and len(box_msg.data) == 4:
                x, y, w, h = box_msg.data
                cv2.rectangle(frame,
                              (int(x), int(y)),
                              (int(x + w), int(y + h)),
                              (0, 255, 0),
                              2)

            frame = cv2.resize(frame, (320, 180))

            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, encoding='jpeg')

            self.pub.publish(compressed_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing annotated frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AnnotatedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()