import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import socket
import os
from dotenv import load_dotenv
from robot_msgs.msg import DetectionArray, TurretState
import time

load_dotenv()

ESP32_IP = os.environ.get("ESP32_IP")
ESP32_PORT = int(os.environ.get("ESP32_PORT"))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SERVO_CENTER = 90
SERVO_MIN = 0
SERVO_MAX = 180
FOV_X = 60.0
FOV_Y = 75.0
MAX_STEP = 3.5
# SMOOTHING = 0.3
DEADZONE = 0.15
FREQUENCY = 60.0

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
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
        self.target_id = -1
        self.resolution = (320, 240)
        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.max_angle = 90.0
        self.last_sent_time = 0.0
        self.update_interval = 1.0 / FREQUENCY

        self.get_logger().info('Tracking Node Setup - Complete')

    def info_callback(self, msg):
        if len(msg.data) >= 2:
            self.resolution = (msg.data[0], msg.data[1])

    def state_callback(self, msg):
        if msg.target_id != -1:
            self.target_id = msg.target_id
        
    def detection_callback(self, msg):
        now = time.time()
        if now - self.last_sent_time < self.update_interval:
            return
        
        tracked_det = None
        for det in msg.detections:
            if det.track_id == self.target_id:
                tracked_det = det
                break
        
        if not tracked_det:
            self.get_logger().debug(f"No detection matches id {self.target_id}")
            return
        
        mid_x = (tracked_det.x1 + tracked_det.x2) / 2
        mid_y = (tracked_det.y1 + tracked_det.y2) / 2

        frame_width, frame_height = self.resolution
        dx = mid_x - frame_width / 2
        dy = mid_y - frame_height / 2

        normalized_dx = dx / (frame_width / 2)
        normalized_dy = dy / (frame_height / 2)
        delta_angle_x = normalized_dx * MAX_STEP
        delta_angle_y = normalized_dy * MAX_STEP

        delta_angle_x = round(delta_angle_x, 3)
        delta_angle_y = round(delta_angle_y, 3)

        delta_angle_x = delta_angle_x if abs(delta_angle_x) > DEADZONE else 0

        udpmsg = f"{delta_angle_x},{delta_angle_y}"
        try:
            sock.sendto(udpmsg.encode(), (ESP32_IP, ESP32_PORT))
            self.last_sent_time = now
            # self.get_logger().info(f"Sent UDP message: {udpmsg} to {ESP32_IP}:{ESP32_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP message: {e}")

            
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()