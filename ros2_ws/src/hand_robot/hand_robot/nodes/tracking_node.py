import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/hand/box',
            self.callback,
            10
        )
        self.resolution = (640, 480) 
        self.create_subscription(Float32MultiArray, '/camera/info', self.info_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, '/motor/cmd', 10)
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

        kx = self.max_angle / (frame_width / 2)
        ky = self.max_angle / (frame_height / 2)

        angle_x = kx * dx
        angle_y = ky * dy

        msg_out = Float32MultiArray(data=[angle_x, angle_y])
        self.motor_pub.publish(msg_out)

        self.get_logger().info(f"Publishing angles -> X: {angle_x:.1f}, Y: {angle_y:.1f}")

            
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()