from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot',
            executable='stream_to_image_node',
            name='stream_to_image_node'
        ),
        Node(
            package='robot',
            executable='prompt_cv_node',
            name='prompt_cv_node'
        ),
        Node(
            package='robot',
            executable='tracking_node',
            name='tracking_node'
        ),
        Node(
            package='robot',
            executable='annotated_publisher',
            name='annotated_publisher'
        ),
        Node(
            package='robot',
            executable='mot_node',
            name='mot_node'
        ),
        Node(
            package='robot',
            executable='sot_node',
            name='sot_node'
        ),
        Node(
            package='robot',
            executable='state_manager',
            name='state_manager'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090, 'address': '0.0.0.0', 'allow_origin': '*'}]
        )
    ])