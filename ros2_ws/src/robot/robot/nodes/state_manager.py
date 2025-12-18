import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, TurretEvent, Input
import copy

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.state = TurretState()
        self.state.mode = "IDLE"
        self.state.status = "IDLE"
        self.state.prompt = ""
        self.state.target_id = -1
        self.state.stamp = self.get_clock().now().to_msg()

        self.state_pub = self.create_publisher(TurretState, '/turret/state', 10)

        self.input_sub = self.create_subscription(Input, '/input', self.input_callback, 10)

        self.event_sub = self.create_subscription(
            TurretEvent,
            '/turret/event',
            self.event_callback,
            10
        )

        self.get_logger().info('StateManager started')

    def input_callback(self, msg):
        prev_prompt = self.state.prompt
        prev_target_id = self.state.target_id
        prev_mode = self.state.mode
        prev_status = self.state.status

        if msg.mode:
            self.state.mode = msg.mode
            if msg.mode in ['IDLE', 'MANUAL']:
                self.state.status = 'IDLE'
            elif msg.mode in ['TRACK', 'AUTO_LOG', 'AUTO_TRACK']:
                self.state.status = 'SEARCHING'

        if msg.clear_prompt:
            self.state.prompt = ""
        elif msg.prompt != "":
            self.state.prompt = msg.prompt

        if msg.clear_target_id:
            self.state.target_id = -1
            self.state.status = 'SEARCHING'
        elif msg.target_id != -1:
            self.state.target_id = msg.target_id
            self.state.status = 'TRACKING'

        self.state.stamp = self.get_clock().now().to_msg()
        if (self.state.prompt != prev_prompt or
            self.state.target_id != prev_target_id or 
            self.state.mode != prev_mode or
            self.state.status != prev_status):
            self.get_logger().info("Input publishing state: " + str(self.state))
            self.state_pub.publish(self.state)
        
    def event_callback(self, msg: TurretEvent):
        prev_prompt = self.state.prompt
        prev_target_id = self.state.target_id
        prev_status = self.state.status

        if msg.clear_prompt:
            self.state.prompt = ""
        elif msg.prompt != "":
            self.state.prompt = msg.prompt
        
        if msg.clear_target_id:
            self.state.target_id = -1
            self.state.status = "SEARCHING"
        elif msg.target_id != -1:
            self.state.target_id = msg.target_id
            self.state.status = "TRACKING"

        if prev_status == "TRACKING" and msg.event == "lost_object":
            self.state.status = "SEARCHING"
            self.state.target_id = -1

        self.state.stamp = self.get_clock().now().to_msg()

        if (self.state.prompt != prev_prompt or
            self.state.target_id != prev_target_id or 
            self.state.status != prev_status):
            self.get_logger().info("Event publishing state: " + str(self.state))
            self.state_pub.publish(self.state)

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()