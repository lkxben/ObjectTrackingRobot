import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, Detection, TurretEvent
from std_msgs.msg import Float32
import time

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
        self.mode = None
        self.status = None
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
        self.motor_pub = self.create_publisher(Float32, '/motor/cmd', 10)

        self.create_timer(1.0 / UPDATE_FREQ, self.update_scan)

    def state_callback(self, msg: TurretState):
        self.mode = msg.mode
        self.status = msg.status

    def detection_callback(self, msg: Detection):
        pass

    def update_scan(self):
        if self.mode not in ['AUTO_LOG', 'AUTO_TRACK'] or self.status != 'SEARCHING':
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
        msg = Float32()
        msg.data = delta_x

        udpmsg = f"{delta_x},{delta_y}"
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()