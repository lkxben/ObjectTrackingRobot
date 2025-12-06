import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, Detection, TurretEvent
import socket
import os
import time
from dotenv import load_dotenv

load_dotenv()

ESP32_IP = os.environ.get("ESP32_IP")
ESP32_PORT = int(os.environ.get("ESP32_PORT"))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UPDATE_FREQ = 20.0
SERVO_SPEED = 5.0
SERVO_MIN = 0.0
SERVO_MAX = 180.0
CENTER_ANGLE = 90.0
RESET_THRESHOLD = 2.0
SWEEP_RESET_COUNT = 10 

class ScanningNode(Node):
    def __init__(self):
        super().__init__('scanning_node')
        self.mode = 'IDLE'
        self.active_target_id = -1
        self.servo_angle = CENTER_ANGLE
        self.scan_dir = 1
        self.last_sent_time = time.time()
        self.sweep_count = 0

        self.state_sub = self.create_subscription(
            TurretState, '/turret/state', self.state_callback, 10
        )
        self.detection_sub = self.create_subscription(
            Detection, '/detection/overlay', self.detection_callback, 10
        )
        self.event_pub = self.create_publisher(TurretEvent, '/turret/event', 10)

        self.create_timer(1.0 / UPDATE_FREQ, self.update_scan)

    def state_callback(self, msg: TurretState):
        if self.mode != msg.mode and msg.mode == "AUTO":
            try:
                sock.sendto("RESET".encode(), (ESP32_IP, ESP32_PORT))
                self.servo_angle = CENTER_ANGLE
                self.scan_dir = 1
                self.sweep_count = 0
                self.get_logger().info("[SCANNING] AUTO mode started, servo reset to center")
            except Exception as e:
                self.get_logger().error(f"Failed to send RESET on AUTO start: {e}")
        self.mode = msg.mode
        self.active_target_id = msg.target_id

    def detection_callback(self, msg: Detection):
        pass

    def update_scan(self):
        if self.mode != 'AUTO' or self.active_target_id != -1:
            return

        now = time.time()
        dt = now - self.last_sent_time
        if dt <= 0.0:
            return

        delta_angle = SERVO_SPEED * dt * self.scan_dir
        self.servo_angle += delta_angle
        if self.servo_angle >= SERVO_MAX:
            self.servo_angle = SERVO_MAX
            self.scan_dir = -1
            self.sweep_count += 1
            delta_angle = 0
        elif self.servo_angle <= SERVO_MIN:
            self.servo_angle = SERVO_MIN
            self.scan_dir = 1
            self.sweep_count += 1
            delta_angle = 0

        delta_x = round(delta_angle, 3)
        delta_y = 0.0

        udpmsg = f"{delta_x},{delta_y}"
        try:
            sock.sendto(udpmsg.encode(), (ESP32_IP, ESP32_PORT))
            self.last_sent_time = now
            self.get_logger().info(f"[SCANNING] Sent UDP message: {udpmsg}")
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP message: {e}")

        if self.sweep_count >= SWEEP_RESET_COUNT and abs(self.servo_angle - CENTER_ANGLE) <= RESET_THRESHOLD:
            try:
                sock.sendto("RESET".encode(), (ESP32_IP, ESP32_PORT))
                self.servo_angle = CENTER_ANGLE
                self.sweep_count = 0
                self.get_logger().info("[SCANNING] Servo reset to center (timed)")
            except Exception as e:
                self.get_logger().error(f"Failed to send RESET message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()