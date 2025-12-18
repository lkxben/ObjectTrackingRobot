import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
import struct

SERVO_MIN = 0.0
SERVO_MAX = 180.0
CENTER_ANGLE = 90.0
UPDATE_FREQ = 50.0
DEADBAND = 0.05

USB_PORT = '/dev/ttyACM0'
BAUDRATE = 460800

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.servo_angle = CENTER_ANGLE
        self.delta_cmd = 0.0

        try:
            self.ser = serial.Serial(USB_PORT, BAUDRATE, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f"Connected to ESP32 on {USB_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self.ser = None

        self.send_angle(self.servo_angle)
        self.motor_sub = self.create_subscription(
            Float32, '/motor/cmd', self.motor_callback, 10
        )
        self.pos_pub = self.create_publisher(Float32, '/turret/position', 10)
        self.timer = self.create_timer(1.0 / UPDATE_FREQ, self.update_servo)
        self.get_logger().info(f'Motor Node Started')

    def motor_callback(self, msg):
        self.delta_cmd += msg.data

    def send_angle(self, angle):
        if self.ser is None:
            return
        angle = max(SERVO_MIN, min(SERVO_MAX, angle))
        encoded = round(angle / 180.0 * 65535)
        try:
            self.ser.write(b'\xFF' + struct.pack('>H', encoded))
        except Exception as e:
            self.get_logger().error(f"Failed to send angle: {e}")

    def update_servo(self):
        if abs(self.delta_cmd) >= DEADBAND:
            self.servo_angle += self.delta_cmd
            self.servo_angle = max(SERVO_MIN, min(SERVO_MAX, self.servo_angle))
            self.delta_cmd = 0.0
        self.send_angle(self.servo_angle)

        msg = Float32()
        msg.data = self.servo_angle
        self.pos_pub.publish(msg)

    def destroy_node(self):
        self.send_angle(self.SERVO_CENTER)
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()