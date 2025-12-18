import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, TurretEvent
from std_msgs.msg import Float32

UPDATE_FREQ = 50.0
STEP = 0.05
SERVO_MIN = 0.0
SERVO_MAX = 180.0

class ScanningNode(Node):
    def __init__(self):
        super().__init__('scanning_node')
        self.mode = None
        self.status = None
        self.delta_angle = STEP
        self.last_pos = 90.0

        self.state_sub = self.create_subscription(
            TurretState, '/turret/state', self.state_callback, 10
        )
        self.pos_sub = self.create_subscription(
            Float32, '/turret/position', self.position_callback, 10
        )
        self.event_pub = self.create_publisher(TurretEvent, '/turret/event', 10)
        self.motor_pub = self.create_publisher(Float32, '/motor/cmd', 10)

        self.create_timer(1.0 / UPDATE_FREQ, self.update_scan)

    def state_callback(self, msg: TurretState):
        self.mode = msg.mode
        self.status = msg.status

    def position_callback(self, msg):
        self.last_pos = msg.data
        if self.last_pos >= SERVO_MAX and self.delta_angle > 0:
            self.delta_angle *= -1
        elif self.last_pos <= SERVO_MIN and self.delta_angle < 0:
            self.delta_angle *= -1

    def update_scan(self):
        if self.mode not in ['AUTO_LOG', 'AUTO_TRACK'] or self.status != 'SEARCHING':
            return

        msg = Float32()
        msg.data = self.delta_angle
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()