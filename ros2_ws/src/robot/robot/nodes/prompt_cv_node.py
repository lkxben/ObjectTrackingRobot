import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
from ultralytics import YOLOE
from robot_msgs.msg import Detection, DetectionArray

class PromptCVNode(Node):
    def __init__(self):
        super().__init__('prompt_cv_node')
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.prompt_sub = self.create_subscription(
            String,
            '/prompt/input',
            self.prompt_callback,
            10
        )
        self.det_pub = self.create_publisher(DetectionArray, '/detection', 10)

        self.bridge = CvBridge()
        self.model_pf = YOLOE("yoloe-11s-seg-pf.pt")
        self.model_prompt = YOLOE("yoloe-11s-seg.pt")
        self.model = self.model_pf
        self.prompts = None
        _ = self.model_prompt.get_text_pe(["warmup"])

        self.get_logger().info('Prompt CV Setup - Complete')

    def prompt_callback(self, msg):
        if msg.data is "clear":
            self.prompts = None
            self.model = self.model_pf
            self.get_logger().info("All mode: detecting all classes")
        elif msg.data.strip():  # non-empty string
            self.prompts = msg.data.split(",")
            self.model = self.model_prompt
            self.model.set_classes(self.prompts, self.model_prompt.get_text_pe(self.prompts))
            self.get_logger().info(f"Prompt mode: detecting {self.prompts}")
        else:  # empty string
            self.prompts = None
            self.model = self.model_pf
            self.get_logger().info("All mode: detecting all classes")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)

        results = self.model.predict(source=frame, imgsz=320, conf=0.35, verbose=False)
        detections_msg = DetectionArray()

        boxes = results[0].boxes
        if len(boxes) == 0:
            self.det_pub.publish(detections_msg)
            return

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy()

        for i in range(len(boxes)):
            x1, y1, x2, y2 = xyxy[i]
            conf = float(confs[i])
            class_id = int(class_ids[i])
            class_name = results[0].names[class_id]

            det = Detection()
            det.x1 = float(x1)
            det.y1 = float(y1)
            det.x2 = float(x2)
            det.y2 = float(y2)
            det.confidence = conf
            det.class_name = class_name
            det.class_id = class_id

            detections_msg.detections.append(det)
            
        self.det_pub.publish(detections_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PromptCVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()