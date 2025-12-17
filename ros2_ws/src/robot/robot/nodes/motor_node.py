import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, Detection, TurretEvent
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

SERVO_PIN = 14       # GPIO pin for pan servo
SERVO_MIN = 0.0
SERVO_MAX = 180.0
CENTER_ANGLE = 90.0
UPDATE_FREQ = 20.0

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.mode = None
        self.status = None
        self.servo_angle = CENTER_ANGLE
        self.delta_cmd = 0.0
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for standard servo
        self.servo_pwm.start(self.angle_to_duty(self.servo_angle))

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

    def angle_to_duty(self, angle):
        return 2.5 + (angle / 180.0) * 10.0

    def motor_callback(self, msg):
        self.delta_cmd += msg.data

    def update_servo(self):
        self.servo_angle += self.delta_cmd
        self.servo_angle = max(SERVO_MIN, min(SERVO_MAX, self.servo_angle))
        self.servo_pwm.ChangeDutyCycle(self.angle_to_duty(self.servo_angle))
        self.delta_cmd = 0.0

        msg = Float32()
        msg.data = self.servo_angle
        self.pos_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.servo_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

