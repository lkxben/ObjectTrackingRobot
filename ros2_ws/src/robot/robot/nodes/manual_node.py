import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, TurretEvent
from std_msgs.msg import Float32

class ManualNode(Node):
    def __init__(self):
        super().__init__('manual_node')
        self.mode = None

        self.state_sub = self.create_subscription(
            TurretState, '/turret/state', self.state_callback, 10
        )
        self.man_sub = self.create_subscription(
            Float32, '/motor/manual', self.manual_callback, 10
        )
        self.motor_pub = self.create_publisher(Float32, '/motor/cmd', 10)
        self.get_logger().info("Manual Node Started")

    def state_callback(self, msg: TurretState):
        self.mode = msg.mode

    def manual_callback(self, msg: Float32):
        if self.mode != "MANUAL":
            return
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManualNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()