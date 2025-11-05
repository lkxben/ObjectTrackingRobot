from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hand_robot',
            executable='stream_to_image_node',
            name='stream_to_image_node'
        ),
        Node(
            package='hand_robot',
            executable='hand_pose_node',
            name='hand_pose_node'
        ),
        Node(
            package='hand_robot',
            executable='prompt_cv_node',
            name='prompt_cv_node'
        ),
        Node(
            package='hand_robot',
            executable='tracking_node',
            name='tracking_node'
        ),
        Node(
            package='hand_robot',
            executable='annotated_publisher',
            name='annotated_publisher'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090, 'address': '0.0.0.0', 'allow_origin': '*'}]
        )
    ])