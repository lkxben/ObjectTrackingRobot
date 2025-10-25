import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO

class HandPoseNode(Node):
    def __init__(self):
        super().__init__('hand_pose_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, '/hand/pose', 10)
        self.bridge = CvBridge()
        self.model = YOLO("weights/best.pt")

    def image_callback(self, msg):
        self.get_logger().info('Received image frame')
        frame = self.bridge.imgmsg_to_cv2(msg)
        
        results = self.model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)

        for result in results:
            if result.keypoints is None or result.keypoints.data.shape[0] == 0:
                continue  # No detection
            confs = result.keypoints.conf
            avg_conf = confs.mean().item()
            if avg_conf > best_conf:
                best_conf = avg_conf
                best_kpts = result.keypoints.data[0].cpu().numpy()

        if best_kpts is not None:
            msg_out = Float32MultiArray(data=best_kpts)
            self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = HandPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()