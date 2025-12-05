import rclpy
from rclpy.node import Node
from robot_msgs.msg import TurretState, TurretEvent, Input
import copy

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.state = TurretState()
        self.state.mode = "IDLE"
        # self.state.status = "IDLE"
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
        self.state_pub.publish(self.state)

        self.get_logger().info('StateManager started')

    def input_callback(self, msg):
        if msg.mode:
            self.state.mode = msg.mode
        if msg.prompt != "__NONE__":
            self.state.prompt = "" if msg.prompt == "__EMPTY__" else msg.prompt
        if msg.target_id != -1:
            self.state.target_id = -1 if msg.target_id == -999 else msg.target_id
        self.state.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.state)
        
    def event_callback(self, msg: TurretEvent):
        prev_state = copy.deepcopy(self.state)

        if msg.prompt:
            self.state.prompt = msg.prompt
        else:
            self.state.prompt = prev_state.prompt
        
        if msg.target_id != -1:
            self.state.target_id = msg.target_id
        else:
            self.state.target = prev_state.prompt

        # self.update_status(prev_state, msg)

        self.state.stamp = self.get_clock().now().to_msg()
        self.state_pub.publish(self.state)

    # def update_status(self, prev_state: TurretState, event: TurretEvent):
    #     mode = self.state.mode
    #     status = self.state.status

    #     # IDLE mode
    #     if mode == "IDLE":
    #         if status == "IDLE":
    #             self.state.status = "DETECTING"
    #         elif status == "DETECTING":
    #             self.state.status = "LOGGING"
    #         elif status == "LOGGING":
    #             self.state.status = "IDLE"

    #     # MANUAL mode
    #     elif mode == "MANUAL":
    #         self.state.status = "MANUAL_CONTROL"

    #     # TRACK mode
    #     elif mode == "TRACK":
    #         if status == "DETECTING":
    #             self.state.status = "TRACKING"
    #         elif status == "TRACKING":
    #             self.state.status = "LOST"

    #     # AUTO mode
    #     elif mode == "AUTO":
    #         if self.state.prompt and self.state.target_id:
    #             # prompt + track
    #             if status == "SEARCHING":
    #                 self.state.status = "DETECTING"
    #             elif status == "DETECTING":
    #                 self.state.status = "TRACKING"
    #             elif status == "TRACKING":
    #                 self.state.status = "LOST"
    #             elif status == "LOST":
    #                 self.state.status = "SEARCHING"
    #         elif self.state.prompt and not self.state.target_id:
    #             # prompt only
    #             if status == "SEARCHING":
    #                 self.state.status = "DETECTING"
    #             elif status == "DETECTING":
    #                 self.state.status = "LOGGING"
    #             elif status == "LOGGING":
    #                 self.state.status = "SEARCHING"
    #         else:
    #             # no prompt
    #             if status == "SEARCHING":
    #                 self.state.status = "DETECTING"
    #             elif status == "DETECTING":
    #                 self.state.status = "LOGGING"
    #             elif status == "LOGGING":
    #                 self.state.status = "SEARCHING"

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()