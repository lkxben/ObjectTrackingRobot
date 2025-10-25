import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class FakeCameraNode(Node):
    def __init__(self):
        super().__init__('fake_camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Path to your test image
        image_path = os.path.join(os.path.dirname(__file__), 'hand_br.png')
        self.img = cv2.imread(image_path)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 FPS

    def timer_callback(self):
        if self.img is not None:
            msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing simulated image')

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()