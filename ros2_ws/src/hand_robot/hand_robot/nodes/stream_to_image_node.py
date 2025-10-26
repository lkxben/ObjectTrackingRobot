import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import os
from cv_bridge import CvBridge
from dotenv import load_dotenv

load_dotenv()

class StreamToImageNode(Node):
    def __init__(self):
        super().__init__('stream_to_image_node')
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(Float32MultiArray, '/camera/info', 10)
        self.bridge = CvBridge()

        stream_url = os.environ.get("CAMERA_URL", "http://127.0.0.1:5002/video") # change to esp32 later
        self.cap = cv2.VideoCapture(stream_url)

        if not self.cap.isOpened():
            self.get_logger().error("Camera not found!")
            return

        self.get_logger().info('Connected to stream')

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.timer = self.create_timer(1/30.0, self.publish_frame)
        self.info_timer = self.create_timer(1.0, self.publish_info)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(msg)

    def publish_info(self):
        msg = Float32MultiArray(data=[self.width, self.height])
        self.info_pub.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StreamToImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()