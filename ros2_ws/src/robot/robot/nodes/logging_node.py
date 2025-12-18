import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretEvent, TurretLog, TurretState

class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')
        self.log_pub = self.create_publisher(TurretLog, '/turret/log', 10)
        self.event_sub = self.create_subscription(
            TurretEvent,
            '/turret/event',
            self.event_callback,
            10
        )
        self.state_sub = self.create_subscription(
            TurretState,
            '/turret/state',
            self.state_callback,
            10
        )
        self.state = TurretState()
        self.get_logger().info("Logging Node started")

    def state_callback(self, msg: TurretState):
        self.state = msg
        debug_msg = TurretLog()
        debug_msg.level = "debug"
        debug_msg.message = f"[MODE]: {self.state.mode} [STATUS]: {self.state.status}"
        
        self.log_pub.publish(debug_msg)

    def event_callback(self, msg: TurretEvent):
        log_msg = TurretLog()
        log_msg.stamp = self.get_clock().now().to_msg()

        log_msg.level = "info"

        if msg.event == "lost_object":
            log_msg.level = "warning"
            log_msg.message = f"Lost track of '{self.state.prompt}' (ID: {self.state.target_id})"
        elif msg.event == "tracking_object":
            log_msg.message= f"Tracking '{self.state.prompt}' (ID: {self.state.target_id})"
        elif msg.event == "object_detected":
            log_msg.message = f"Detected '{msg.message}'"
        else:
            log_msg.message = f"{msg.event}: {msg.message}"
        
        self.log_pub.publish(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()