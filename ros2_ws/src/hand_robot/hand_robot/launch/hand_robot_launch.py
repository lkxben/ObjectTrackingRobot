from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_robot',
            executable='fake_camera_node',
            name='fake_camera_node'
        ),
        Node(
            package='hand_robot',
            executable='hand_pose_node',
            name='hand_pose_node'
        ),
        Node(
            package='hand_robot',
            executable='tracking_node',
            name='tracking_node'
        )
    ])