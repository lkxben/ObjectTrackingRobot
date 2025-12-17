import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, Detection, TurretEvent
from std_msgs.msg import Float32
from gpiozero import AngularServo

SERVO_PIN = 12       # GPIO pin for pan servo
SERVO_MIN = 0.0
SERVO_MAX = 180.0
CENTER_ANGLE = 90.0
UPDATE_FREQ = 15.0
DEADBAND = 0.05

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.mode = None
        self.status = None
        self.servo_angle = CENTER_ANGLE
        self.delta_cmd = 0.0
        
        self.servo = AngularServo(
            SERVO_PIN,
            min_angle=SERVO_MIN,
            max_angle=SERVO_MAX,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )
        self.servo.angle = self.servo_angle

        # self.state_sub = self.create_subscription(
        #     TurretState, '/turret/state', self.state_callback, 10
        # )
        self.motor_sub = self.create_subscription(
            Float32, '/motor/cmd', self.motor_callback, 10
        )
        self.event_pub = self.create_publisher(TurretEvent, '/turret/event', 10)
        self.pos_pub = self.create_publisher(Float32, '/turret/position', 10)
        self.timer = self.create_timer(1.0 / UPDATE_FREQ, self.update_servo)
        self.get_logger().info(f'Motor node initialized at {self.servo_angle}°')

    # def state_callback(self, msg: TurretState):
    #     self.mode = msg.mode
    #     self.status = msg.status

    def motor_callback(self, msg):
        self.delta_cmd += msg.data

    def update_servo(self):
        if self.delta_cmd < DEADBAND:
            self.delta_cmd = 0.0
            self.servo.detach()
            return
        
        if self.servo.value is None:
            self.servo.value = 0

        self.servo_angle += self.delta_cmd
        self.servo_angle = max(SERVO_MIN, min(SERVO_MAX, self.servo_angle))
        self.delta_cmd = 0.0
        self.servo.angle = self.servo_angle

        msg = Float32()
        msg.data = self.servo_angle
        self.pos_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.servo.angle = CENTER_ANGLE

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

