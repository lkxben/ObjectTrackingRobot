import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import websocket
import json
import base64
import os
from dotenv import load_dotenv

load_dotenv()

local_host = os.environ.get("LOCAL_HOST", "127.0.0.1")
port = os.environ.get("WEBSOCKET_PORT", 8080)
ws_url = os.environ.get("WS_URL", f'ws://{local_host}:{port}')

class StreamingNode(Node):
    def __init__(self):
        super().__init__('streaming_image_node')
        self.get_logger().info('Started streaming node')
        self.bridge = CvBridge()
        self.box_sub = self.create_subscription(
            Float32MultiArray,
            '/hand/box',
            self.box_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(ws_url)
        try:
            self.ws = websocket.WebSocket()
            self.ws.connect(ws_url) 
            self.get_logger().info('Connected to WebSocket server')
        except Exception as e:
            self.get_logger().error(f'Could not connect to WebSocket server: {e}')
            self.ws = None

    def box_callback(self, msg):
        self.latest_box = msg.data

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', frame)
            b64_frame = base64.b64encode(jpeg.tobytes()).decode('utf-8')

            payload = {
                "type": "image",
                "data": b64_frame
            }

            if self.ws:
                self.ws.send(json.dumps(payload))     
        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')        

def main(args=None):
    rclpy.init(args=args)
    node = StreamingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()