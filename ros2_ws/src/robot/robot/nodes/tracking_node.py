import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import os
from dotenv import load_dotenv

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
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/prompt/box',
            self.callback,
            10
        )
        self.resolution = (640, 480) 
        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        # self.motor_pub = self.create_publisher(Float32MultiArray, '/motor/cmd', 10)
        self.max_angle = 90.0

    def info_callback(self, msg):
        self.resolution = (msg.data[0], msg.data[1])
        
    def callback(self, msg):
        data = msg.data
        if len(data) < 4:
            self.get_logger().warn('Received box with insufficient data')
            return
        
        mid_x = (data[0] + data[2]) / 2
        mid_y = (data[1] + data[3]) / 2

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
        # msg_out = Float32MultiArray(data=[angle_x, angle_y])
        # self.motor_pub.publish(msg_out)
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