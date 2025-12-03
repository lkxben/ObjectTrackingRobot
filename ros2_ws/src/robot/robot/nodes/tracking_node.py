import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import socket
import os
from dotenv import load_dotenv
from robot_msgs.msg import DetectionArray

load_dotenv()

ESP32_IP = os.environ.get("ESP32_IP")
ESP32_PORT = int(os.environ.get("ESP32_PORT"))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SERVO_CENTER = 90
SERVO_MIN = 0
SERVO_MAX = 180
FOV_X = 60.0
FOV_Y = 75.0
MAX_STEP = 5.0
SMOOTHING = 0.5
DEADZONE = 1.3

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
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
        self.resolution = (320, 240)
        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.max_angle = 90.0
        self.get_logger().info('Tracking Node Setup - Complete')

    def info_callback(self, msg):
        self.resolution = (msg.data[0], msg.data[1])

    def trackId_callback(self, msg):
        self.get_logger().info(msg.data)
        self.trackId = msg.data
        
    def image_callback(self, msg):
        tracked_det = None
        for det in msg.detections:
            if det.track_id == self.trackId:
                tracked_det = det
                break
        
        if not tracked_det:
            self.get_logger().debug(f"No detection matches id {self.trackId}")
        
        mid_x = (tracked_det.x1 + tracked_det.x2) / 2
        mid_y = (tracked_det.y1 + tracked_det.y2) / 2

        frame_width, frame_height = self.resolution
        dx = mid_x - frame_width / 2
        dy = mid_y - frame_height / 2

        delta_angle_x = (dx / frame_width) * FOV_X
        delta_angle_y = (dy / frame_height) * FOV_Y

        delta_angle_x = max(min(delta_angle_x, MAX_STEP), -MAX_STEP)
        delta_angle_y = max(min(delta_angle_y, MAX_STEP), -MAX_STEP)

        delta_angle_x *= SMOOTHING
        delta_angle_y *= SMOOTHING

        delta_angle_x = round(delta_angle_x, 1)
        delta_angle_y = round(delta_angle_y, 1)

        delta_angle_x = delta_angle_x if abs(delta_angle_x) > DEADZONE else 0

        udpmsg = f"{delta_angle_x},{delta_angle_y}"
        try:
            sock.sendto(udpmsg.encode(), (ESP32_IP, ESP32_PORT))
            # self.get_logger().info(f"Sent UDP message: {udpmsg} to {ESP32_IP}:{ESP32_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP message: {e}")

            
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()