import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLOE

class PromptCVNode(Node):
    def __init__(self):
        super().__init__('prompt_cv_node')
        self.get_logger().info('Prompt CV Setup - Started')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.box_pub = self.create_publisher(Float32MultiArray, '/prompt/box', 10)

        self.bridge = CvBridge()
        self.prompts = ["person"]
        self.model = YOLOE("yoloe-11s-seg.pt")
        self.model.set_classes(self.prompts, self.model.get_text_pe(self.prompts))
        self.get_logger().info('Prompt CV Setup - Complete')

    def prompt_callback(self, msg):
        self.prompts = msg.data.split(",")
        self.model.set_classes(self.prompts, self.model.get_text_pe(self.prompts))
        self.get_logger().info(f"Updated prompts: {self.prompts}")

    def image_callback(self, msg):
        # self.get_logger().info('Received image frame')
        frame = self.bridge.imgmsg_to_cv2(msg)
        
        results = self.model.predict(source=frame, imgsz=320, conf=0.2, verbose=False)

        if len(results[0].boxes) > 0:
            box = results[0].boxes.xyxy[0].cpu().numpy()
            box_msg = Float32MultiArray(data=box.tolist())
            self.box_pub.publish(box_msg)
            
            center_x = (box[0] + box[2]) / 2
            center_y = (box[1] + box[3]) / 2
        #     self.get_logger().info(f"Published box center: ({center_x:.1f}, {center_y:.1f})")
        # else:
        #     self.get_logger().info("No objects detected")

def main(args=None):
    rclpy.init(args=args)
    node = PromptCVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()