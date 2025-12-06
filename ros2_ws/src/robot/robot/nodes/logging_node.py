import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretEvent, TurretLog

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
        self.get_logger().info("Logging Node started")

    def event_callback(self, msg: TurretEvent):
        log_msg = TurretLog()
        log_msg.stamp = self.get_clock().now().to_msg()

        if msg.event in ["lost_object"]:
            log_msg.level = "warning"
        else:
            log_msg.level = "info"

        if msg.event == "lost_object":
            log_msg.message = f"Lost track of {msg.message} object"
        if msg.event == "tracking_object":
            log_msg.message= f"Tracking {msg.message} object with ID: {msg.target_id}"
        if msg.event == "object_detected":
            log_msg.message = f"{msg.message} object detected"
        self.get_logger().info(str(log_msg))
        self.log_pub.publish(log_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()